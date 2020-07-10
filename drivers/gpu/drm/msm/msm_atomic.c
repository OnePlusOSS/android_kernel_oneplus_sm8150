/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2014 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/msm_drm_notify.h>
#include <linux/notifier.h>

#include "msm_drv.h"
#include "msm_kms.h"
#include "msm_gem.h"
#include "msm_fence.h"
#include "sde_trace.h"

#define MULTIPLE_CONN_DETECTED(x) (x > 1)

struct msm_commit {
	struct drm_device *dev;
	struct drm_atomic_state *state;
	uint32_t crtc_mask;
	bool nonblock;
	struct kthread_work commit_work;
};

static BLOCKING_NOTIFIER_HEAD(msm_drm_notifier_list);
int connector_state_crtc_index;
/**
 * msm_drm_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *
 * This function registers a notifier callback function
 * to msm_drm_notifier_list, which would be called when
 * received unblank/power down event.
 */
int msm_drm_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&msm_drm_notifier_list,
						nb);
}
EXPORT_SYMBOL(msm_drm_register_client);

/**
 * msm_drm_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *
 * This function unregisters the callback function from
 * msm_drm_notifier_list.
 */
int msm_drm_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&msm_drm_notifier_list,
						  nb);
}
EXPORT_SYMBOL(msm_drm_unregister_client);

/**
 * msm_drm_notifier_call_chain - notify clients of drm_events
 * @val: event MSM_DRM_EARLY_EVENT_BLANK or MSM_DRM_EVENT_BLANK
 * @v: notifier data, inculde display id and display blank
 *     event(unblank or power down).
 */
int msm_drm_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&msm_drm_notifier_list, val,
					    v);
}
EXPORT_SYMBOL(msm_drm_notifier_call_chain);

/* block until specified crtcs are no longer pending update, and
 * atomically mark them as pending update
 */
static int start_atomic(struct msm_drm_private *priv, uint32_t crtc_mask)
{
	int ret;

	spin_lock(&priv->pending_crtcs_event.lock);
	ret = wait_event_interruptible_locked(priv->pending_crtcs_event,
			!(priv->pending_crtcs & crtc_mask));
	if (ret == 0) {
		DBG("start: %08x", crtc_mask);
		priv->pending_crtcs |= crtc_mask;
	}
	spin_unlock(&priv->pending_crtcs_event.lock);

	return ret;
}

/* clear specified crtcs (no longer pending update)
 */
static void end_atomic(struct msm_drm_private *priv, uint32_t crtc_mask)
{
	spin_lock(&priv->pending_crtcs_event.lock);
	DBG("end: %08x", crtc_mask);
	priv->pending_crtcs &= ~crtc_mask;
	wake_up_all_locked(&priv->pending_crtcs_event);
	spin_unlock(&priv->pending_crtcs_event.lock);
}

static void commit_destroy(struct msm_commit *c)
{
	end_atomic(c->dev->dev_private, c->crtc_mask);
	if (c->nonblock)
		kfree(c);
}

static inline bool _msm_seamless_for_crtc(struct drm_atomic_state *state,
			struct drm_crtc_state *crtc_state, bool enable)
{
	struct drm_connector *connector = NULL;
	struct drm_connector_state  *conn_state = NULL;
	int i = 0;
	int conn_cnt = 0;

	if (msm_is_mode_seamless(&crtc_state->mode) ||
		msm_is_mode_seamless_vrr(&crtc_state->adjusted_mode) ||
		msm_is_mode_seamless_dyn_clk(&crtc_state->adjusted_mode))
		return true;

	if (msm_is_mode_seamless_dms(&crtc_state->adjusted_mode) && !enable)
		return true;

	if (!crtc_state->mode_changed && crtc_state->connectors_changed) {
		for_each_connector_in_state(state, connector, conn_state, i) {
			if ((conn_state->crtc == crtc_state->crtc) ||
					(connector->state->crtc ==
					 crtc_state->crtc))
				conn_cnt++;

			if (MULTIPLE_CONN_DETECTED(conn_cnt))
				return true;
		}
	}

	return false;
}

static inline bool _msm_seamless_for_conn(struct drm_connector *connector,
		struct drm_connector_state *old_conn_state, bool enable)
{
	if (!old_conn_state || !old_conn_state->crtc)
		return false;

	if (!old_conn_state->crtc->state->mode_changed &&
			!old_conn_state->crtc->state->active_changed &&
			old_conn_state->crtc->state->connectors_changed) {
		if (old_conn_state->crtc == connector->state->crtc)
			return true;
	}

	if (enable)
		return false;

	if (msm_is_mode_seamless(&connector->encoder->crtc->state->mode))
		return true;

	if (msm_is_mode_seamless_vrr(
			&connector->encoder->crtc->state->adjusted_mode))
		return true;

	if (msm_is_mode_seamless_dyn_clk(
			 &connector->encoder->crtc->state->adjusted_mode))
		return true;

	if (msm_is_mode_seamless_dms(
			&connector->encoder->crtc->state->adjusted_mode))
		return true;

	return false;
}

static void msm_atomic_wait_for_commit_done(
		struct drm_device *dev,
		struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	struct msm_drm_private *priv = old_state->dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int i;

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		if (!new_crtc_state->active)
			continue;

		if (drm_crtc_vblank_get(crtc))
			continue;

		kms->funcs->wait_for_crtc_commit_done(kms, crtc);

		drm_crtc_vblank_put(crtc);
	}
}

static void
msm_disable_outputs(struct drm_device *dev, struct drm_atomic_state *old_state)
{
	struct drm_connector *connector;
	struct drm_connector_state *old_conn_state;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct msm_drm_notifier notifier_data;
	int i, blank;

	SDE_ATRACE_BEGIN("msm_disable");
	for_each_connector_in_state(old_state, connector, old_conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_encoder *encoder;
		struct drm_crtc_state *old_crtc_state;
		unsigned int crtc_idx;

		/*
		 * Shut down everything that's in the changeset and currently
		 * still on. So need to check the old, saved state.
		 */
		if (!old_conn_state->crtc)
			continue;

		crtc_idx = drm_crtc_index(old_conn_state->crtc);
		old_crtc_state = drm_atomic_get_old_crtc_state(old_state,
							old_conn_state->crtc);

		if (!old_crtc_state->active ||
		    !drm_atomic_crtc_needs_modeset(old_conn_state->crtc->state))
			continue;

		encoder = old_conn_state->best_encoder;

		/* We shouldn't get this far if we didn't previously have
		 * an encoder.. but WARN_ON() rather than explode.
		 */
		if (WARN_ON(!encoder))
			continue;

		if (_msm_seamless_for_conn(connector, old_conn_state, false))
			continue;

		funcs = encoder->helper_private;

		DRM_DEBUG_ATOMIC("disabling [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		if (connector->state->crtc &&
			connector->state->crtc->state->active_changed) {
			blank = MSM_DRM_BLANK_POWERDOWN;
			notifier_data.data = &blank;
			notifier_data.id = crtc_idx;
			connector_state_crtc_index = crtc_idx;
			msm_drm_notifier_call_chain(MSM_DRM_EARLY_EVENT_BLANK,
						     &notifier_data);
		}
		/*
		 * Each encoder has at most one connector (since we always steal
		 * it away), so we won't call disable hooks twice.
		 */
		drm_bridge_disable(encoder->bridge);

		/* Right function depends upon target state. */
		if (connector->state->crtc && funcs->prepare)
			funcs->prepare(encoder);
		else if (funcs->disable)
			funcs->disable(encoder);
		else
			funcs->dpms(encoder, DRM_MODE_DPMS_OFF);

		drm_bridge_post_disable(encoder->bridge);
		if (connector->state->crtc &&
			connector->state->crtc->state->active_changed) {
			DRM_DEBUG_ATOMIC("Notify blank\n");
			msm_drm_notifier_call_chain(MSM_DRM_EVENT_BLANK,
						&notifier_data);
		}
	}

	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		/* Shut down everything that needs a full modeset. */
		if (!drm_atomic_crtc_needs_modeset(crtc->state))
			continue;

		if (!old_crtc_state->active)
			continue;

		if (_msm_seamless_for_crtc(old_state, crtc->state, false))
			continue;

		funcs = crtc->helper_private;

		DRM_DEBUG_ATOMIC("disabling [CRTC:%d]\n",
				 crtc->base.id);

		/* Right function depends upon target state. */
		if (crtc->state->enable && funcs->prepare)
			funcs->prepare(crtc);
		else if (funcs->disable)
			funcs->disable(crtc);
		else
			funcs->dpms(crtc, DRM_MODE_DPMS_OFF);
	}
	SDE_ATRACE_END("msm_disable");
}

static void
msm_crtc_set_mode(struct drm_device *dev, struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *old_conn_state;
	int i;

	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		if (!crtc->state->mode_changed)
			continue;

		funcs = crtc->helper_private;

		if (crtc->state->enable && funcs->mode_set_nofb) {
			DRM_DEBUG_ATOMIC("modeset on [CRTC:%d]\n",
					 crtc->base.id);

			funcs->mode_set_nofb(crtc);
		}
	}

	for_each_connector_in_state(old_state, connector, old_conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_crtc_state *new_crtc_state;
		struct drm_encoder *encoder;
		struct drm_display_mode *mode, *adjusted_mode;

		if (!connector->state->best_encoder)
			continue;

		encoder = connector->state->best_encoder;
		funcs = encoder->helper_private;
		new_crtc_state = connector->state->crtc->state;
		mode = &new_crtc_state->mode;
		adjusted_mode = &new_crtc_state->adjusted_mode;

		if (!new_crtc_state->mode_changed &&
				new_crtc_state->connectors_changed) {
			if (_msm_seamless_for_conn(connector,
					old_conn_state, false))
				continue;
		} else if (!new_crtc_state->mode_changed) {
			continue;
		}

		DRM_DEBUG_ATOMIC("modeset on [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		/*
		 * Each encoder has at most one connector (since we always steal
		 * it away), so we won't call mode_set hooks twice.
		 */
		if (funcs->mode_set)
			funcs->mode_set(encoder, mode, adjusted_mode);

		drm_bridge_mode_set(encoder->bridge, mode, adjusted_mode);
	}
}

/**
 * msm_atomic_helper_commit_modeset_disables - modeset commit to disable outputs
 * @dev: DRM device
 * @old_state: atomic state object with old state structures
 *
 * This function shuts down all the outputs that need to be shut down and
 * prepares them (if required) with the new mode.
 *
 * For compatibility with legacy crtc helpers this should be called before
 * drm_atomic_helper_commit_planes(), which is what the default commit function
 * does. But drivers with different needs can group the modeset commits together
 * and do the plane commits at the end. This is useful for drivers doing runtime
 * PM since planes updates then only happen when the CRTC is actually enabled.
 */
void msm_atomic_helper_commit_modeset_disables(struct drm_device *dev,
		struct drm_atomic_state *old_state)
{
	msm_disable_outputs(dev, old_state);

	drm_atomic_helper_update_legacy_modeset_state(dev, old_state);

	msm_crtc_set_mode(dev, old_state);
}

/**
 * msm_atomic_helper_commit_modeset_enables - modeset commit to enable outputs
 * @dev: DRM device
 * @old_state: atomic state object with old state structures
 *
 * This function enables all the outputs with the new configuration which had to
 * be turned off for the update.
 *
 * For compatibility with legacy crtc helpers this should be called after
 * drm_atomic_helper_commit_planes(), which is what the default commit function
 * does. But drivers with different needs can group the modeset commits together
 * and do the plane commits at the end. This is useful for drivers doing runtime
 * PM since planes updates then only happen when the CRTC is actually enabled.
 */
static void msm_atomic_helper_commit_modeset_enables(struct drm_device *dev,
		struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct drm_crtc_state *new_crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *new_conn_state;
	struct msm_drm_notifier notifier_data;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int bridge_enable_count = 0;
	int i, blank;
	bool splash = false;

	SDE_ATRACE_BEGIN("msm_enable");
	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state,
			new_crtc_state, i) {
		const struct drm_crtc_helper_funcs *funcs;

		/* Need to filter out CRTCs where only planes change. */
		if (!drm_atomic_crtc_needs_modeset(new_crtc_state))
			continue;

		if (!new_crtc_state->active)
			continue;

		if (_msm_seamless_for_crtc(old_state, crtc->state, true))
			continue;

		funcs = crtc->helper_private;

		if (crtc->state->enable) {
			DRM_DEBUG_ATOMIC("enabling [CRTC:%d]\n",
					 crtc->base.id);

			if (funcs->atomic_enable)
				funcs->atomic_enable(crtc, old_crtc_state);
			else
				funcs->commit(crtc);
		}

		if (msm_needs_vblank_pre_modeset(
					&new_crtc_state->adjusted_mode))
			drm_crtc_wait_one_vblank(crtc);

	}

	for_each_new_connector_in_state(old_state, connector,
			new_conn_state, i) {
		const struct drm_encoder_helper_funcs *funcs;
		struct drm_encoder *encoder;
		struct drm_connector_state *old_conn_state;

		if (!new_conn_state->best_encoder)
			continue;

		if (!new_conn_state->crtc->state->active ||
				!drm_atomic_crtc_needs_modeset(
					new_conn_state->crtc->state))
			continue;

		old_conn_state = drm_atomic_get_old_connector_state(
				old_state, connector);
		if (_msm_seamless_for_conn(connector, old_conn_state, true))
			continue;

		encoder = connector->state->best_encoder;
		funcs = encoder->helper_private;

		DRM_DEBUG_ATOMIC("enabling [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		if (kms && kms->funcs && kms->funcs->check_for_splash)
			splash = kms->funcs->check_for_splash(kms);

		if (splash || (connector->state->crtc &&
			connector->state->crtc->state->active_changed)) {
			blank = MSM_DRM_BLANK_UNBLANK;
			notifier_data.data = &blank;
			notifier_data.id =
				connector->state->crtc->index;
			DRM_DEBUG_ATOMIC("Notify early unblank\n");
			connector_state_crtc_index =
				connector->state->crtc->index;
			msm_drm_notifier_call_chain(MSM_DRM_EARLY_EVENT_BLANK,
					    &notifier_data);
		}
		/*
		 * Each encoder has at most one connector (since we always steal
		 * it away), so we won't call enable hooks twice.
		 */
		drm_bridge_pre_enable(encoder->bridge);
		++bridge_enable_count;

		if (funcs->enable)
			funcs->enable(encoder);
		else
			funcs->commit(encoder);
	}

	if (kms && kms->funcs && kms->funcs->commit) {
		DRM_DEBUG_ATOMIC("triggering commit\n");
		kms->funcs->commit(kms, old_state);
	}

	/* If no bridges were pre_enabled, skip iterating over them again */
	if (bridge_enable_count == 0) {
		SDE_ATRACE_END("msm_enable");
		return;
	}

	for_each_new_connector_in_state(old_state, connector,
			new_conn_state, i) {
		struct drm_encoder *encoder;
		struct drm_connector_state *old_conn_state;

		if (!new_conn_state->best_encoder)
			continue;

		if (!new_conn_state->crtc->state->active ||
		    !drm_atomic_crtc_needs_modeset(
				    new_conn_state->crtc->state))
			continue;

		old_conn_state = drm_atomic_get_old_connector_state(
				old_state, connector);
		if (_msm_seamless_for_conn(connector, old_conn_state, true))
			continue;

		encoder = connector->state->best_encoder;

		DRM_DEBUG_ATOMIC("bridge enable enabling [ENCODER:%d:%s]\n",
				 encoder->base.id, encoder->name);

		drm_bridge_enable(encoder->bridge);

		if (splash || (connector->state->crtc &&
			connector->state->crtc->state->active_changed)) {
			DRM_DEBUG_ATOMIC("Notify unblank\n");
			msm_drm_notifier_call_chain(MSM_DRM_EVENT_BLANK,
					    &notifier_data);
		}
	}
	SDE_ATRACE_END("msm_enable");
}

/* The (potentially) asynchronous part of the commit.  At this point
 * nothing can fail short of armageddon.
 */
static void complete_commit(struct msm_commit *c)
{
	struct drm_atomic_state *state = c->state;
	struct drm_device *dev = state->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	drm_atomic_helper_wait_for_fences(dev, state, false);

	kms->funcs->prepare_commit(kms, state);

	msm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_planes(dev, state, 0);

	msm_atomic_helper_commit_modeset_enables(dev, state);

	/* NOTE: _wait_for_vblanks() only waits for vblank on
	 * enabled CRTCs.  So we end up faulting when disabling
	 * due to (potentially) unref'ing the outgoing fb's
	 * before the vblank when the disable has latched.
	 *
	 * But if it did wait on disabled (or newly disabled)
	 * CRTCs, that would be racy (ie. we could have missed
	 * the irq.  We need some way to poll for pipe shut
	 * down.  Or just live with occasionally hitting the
	 * timeout in the CRTC disable path (which really should
	 * not be critical path)
	 */

	msm_atomic_wait_for_commit_done(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);

	kms->funcs->complete_commit(kms, state);

	drm_atomic_state_put(state);

	priv->commit_end_time =  ktime_get(); //commit end time

	commit_destroy(c);
}

static void _msm_drm_commit_work_cb(struct kthread_work *work)
{
	struct msm_commit *commit =  NULL;

	if (!work) {
		DRM_ERROR("%s: Invalid commit work data!\n", __func__);
		return;
	}

	commit = container_of(work, struct msm_commit, commit_work);

	SDE_ATRACE_BEGIN("complete_commit");
	complete_commit(commit);
	SDE_ATRACE_END("complete_commit");
}

static struct msm_commit *commit_init(struct drm_atomic_state *state,
		bool nonblock)
{
	struct msm_commit *c = kzalloc(sizeof(*c), GFP_KERNEL);

	if (!c)
		return NULL;

	c->dev = state->dev;
	c->state = state;
	c->nonblock = nonblock;

	kthread_init_work(&c->commit_work, _msm_drm_commit_work_cb);

	return c;
}

/* Start display thread function */
static void msm_atomic_commit_dispatch(struct drm_device *dev,
		struct drm_atomic_state *state, struct msm_commit *commit)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_crtc *crtc = NULL;
	struct drm_crtc_state *crtc_state = NULL;
	int ret = -EINVAL, i = 0, j = 0;
	bool nonblock;

	/* cache since work will kfree commit in non-blocking case */
	nonblock = commit->nonblock;

	for_each_crtc_in_state(state, crtc, crtc_state, i) {
		for (j = 0; j < priv->num_crtcs; j++) {
			if (priv->disp_thread[j].crtc_id ==
						crtc->base.id) {
				if (priv->disp_thread[j].thread) {
					kthread_queue_work(
						&priv->disp_thread[j].worker,
							&commit->commit_work);
					/* only return zero if work is
					 * queued successfully.
					 */
					ret = 0;
				} else {
					DRM_ERROR(" Error for crtc_id: %d\n",
						priv->disp_thread[j].crtc_id);
				}
				break;
			}
		}
		/*
		 * TODO: handle cases where there will be more than
		 * one crtc per commit cycle. Remove this check then.
		 * Current assumption is there will be only one crtc
		 * per commit cycle.
		 */
		if (j < priv->num_crtcs)
			break;
	}

	if (ret) {
		/**
		 * this is not expected to happen, but at this point the state
		 * has been swapped, but we couldn't dispatch to a crtc thread.
		 * fallback now to a synchronous complete_commit to try and
		 * ensure that SW and HW state don't get out of sync.
		 */
		DRM_ERROR("failed to dispatch commit to any CRTC\n");
		complete_commit(commit);
	} else if (!nonblock) {
		kthread_flush_work(&commit->commit_work);
	}

	/* free nonblocking commits in this context, after processing */
	if (!nonblock)
		kfree(commit);
}

/**
 * drm_atomic_helper_commit - commit validated state object
 * @dev: DRM device
 * @state: the driver state object
 * @nonblock: nonblocking commit
 *
 * This function commits a with drm_atomic_helper_check() pre-validated state
 * object. This can still fail when e.g. the framebuffer reservation fails.
 *
 * RETURNS
 * Zero for success or -errno.
 */
int msm_atomic_commit(struct drm_device *dev,
		struct drm_atomic_state *state, bool nonblock)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_commit *c;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct drm_plane *plane;
	struct drm_plane_state *old_plane_state, *new_plane_state;
	int i, ret;

	if (!priv || priv->shutdown_in_progress) {
		DRM_ERROR("priv is null or shutdwon is in-progress\n");
		return -EINVAL;
	}

	SDE_ATRACE_BEGIN("atomic_commit");
	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret) {
		SDE_ATRACE_END("atomic_commit");
		return ret;
	}

	c = commit_init(state, nonblock);
	if (!c) {
		ret = -ENOMEM;
		goto error;
	}

	/*
	 * Figure out what crtcs we have:
	 */
	for_each_new_crtc_in_state(state, crtc, crtc_state, i)
		c->crtc_mask |= drm_crtc_mask(crtc);

	/*
	 * Figure out what fence to wait for:
	 */
	for_each_oldnew_plane_in_state(state, plane, old_plane_state, new_plane_state, i) {
		if ((new_plane_state->fb != old_plane_state->fb) && new_plane_state->fb) {
			struct drm_gem_object *obj = msm_framebuffer_bo(new_plane_state->fb, 0);
			struct msm_gem_object *msm_obj = to_msm_bo(obj);
			struct dma_fence *fence = reservation_object_get_excl_rcu(msm_obj->resv);

			drm_atomic_set_fence_for_plane(new_plane_state, fence);
		}
	}

	/*
	 * Wait for pending updates on any of the same crtc's and then
	 * mark our set of crtc's as busy:
	 */
	ret = start_atomic(dev->dev_private, c->crtc_mask);
	if (ret)
		goto err_free;

	BUG_ON(drm_atomic_helper_swap_state(state, false) < 0);

	/*
	 * This is the point of no return - everything below never fails except
	 * when the hw goes bonghits. Which means we can commit the new state on
	 * the software side now.
	 *
	 * swap driver private state while still holding state_lock
	 */
	if (to_kms_state(state)->state)
		priv->kms->funcs->swap_state(priv->kms, state);

	/*
	 * Provide the driver a chance to prepare for output fences. This is
	 * done after the point of no return, but before asynchronous commits
	 * are dispatched to work queues, so that the fence preparation is
	 * finished before the .atomic_commit returns.
	 */
	if (priv->kms && priv->kms->funcs && priv->kms->funcs->prepare_fence)
		priv->kms->funcs->prepare_fence(priv->kms, state);

	/*
	 * Everything below can be run asynchronously without the need to grab
	 * any modeset locks at all under one conditions: It must be guaranteed
	 * that the asynchronous work has either been cancelled (if the driver
	 * supports it, which at least requires that the framebuffers get
	 * cleaned up with drm_atomic_helper_cleanup_planes()) or completed
	 * before the new state gets committed on the software side with
	 * drm_atomic_helper_swap_state().
	 *
	 * This scheme allows new atomic state updates to be prepared and
	 * checked in parallel to the asynchronous completion of the previous
	 * update. Which is important since compositors need to figure out the
	 * composition of the next frame right after having submitted the
	 * current layout.
	 */

	drm_atomic_state_get(state);
	msm_atomic_commit_dispatch(dev, state, c);

	SDE_ATRACE_END("atomic_commit");
	return 0;

err_free:
	kfree(c);
error:
	drm_atomic_helper_cleanup_planes(dev, state);
	SDE_ATRACE_END("atomic_commit");
	return ret;
}

struct drm_atomic_state *msm_atomic_state_alloc(struct drm_device *dev)
{
	struct msm_kms_state *state = kzalloc(sizeof(*state), GFP_KERNEL);

	if (!state || drm_atomic_state_init(dev, &state->base) < 0) {
		kfree(state);
		return NULL;
	}

	return &state->base;
}

void msm_atomic_state_clear(struct drm_atomic_state *s)
{
	struct msm_kms_state *state = to_kms_state(s);
	drm_atomic_state_default_clear(&state->base);
	kfree(state->state);
	state->state = NULL;
}

void msm_atomic_state_free(struct drm_atomic_state *state)
{
	kfree(to_kms_state(state)->state);
	drm_atomic_state_default_release(state);
	kfree(state);
}
