/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_onscreenfingerprint.c
** Description : oplus onscreenfingerprint feature
** Version : 1.0
** Date : 2020/04/15
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/15        1.0           Build this moudle
******************************************************************/

#include "sde_crtc.h"
#include "oplus_onscreenfingerprint.h"
#include "oplus_display_private_api.h"

#define DSI_PANEL_OPLUS_DUMMY_VENDOR_NAME  "PanelVendorDummy"
#define DSI_PANEL_OPLUS_DUMMY_MANUFACTURE_NAME  "dummy1024"

bool oplus_pcc_enabled = false;
bool oplus_skip_pcc = false;
bool oplus_fp_mode = false;


struct drm_msm_pcc oplus_save_pcc;

extern int oplus_underbrightness_alpha;
extern int oplus_dimlayer_dither_threshold;
extern u32 oplus_last_backlight;
extern int oplus_dimlayer_hbm;
int oplus_aod_dim_alpha = CUST_A_NO;
extern int oplus_panel_alpha;
extern int hbm_mode;
extern bool oplus_ffl_trigger_finish;

static struct oplus_brightness_alpha brightness_alpha_lut[] = {
	{0, 0xff},
	{1, 0xee},
	{2, 0xe8},
	{3, 0xe6},
	{4, 0xe5},
	{6, 0xe4},
	{10, 0xe0},
	{20, 0xd5},
	{30, 0xce},
	{45, 0xc6},
	{70, 0xb7},
	{100, 0xad},
	{150, 0xa0},
	{227, 0x8a},
	{300, 0x80},
	{400, 0x6e},
	{500, 0x5b},
	{600, 0x50},
	{800, 0x38},
	{1023, 0x18},
};

static struct oplus_brightness_alpha brightness_alpha_lut_dc[] = {
	{0, 0xff},
	{1, 0xE0},
	{2, 0xd6},
	{3, 0xd5},
	{4, 0xcf},
	{5, 0xcb},
	{6, 0xc9},
	{8, 0xc5},
	{10, 0xc1},
	{15, 0xb6},
	{20, 0xac},
	{30, 0x9d},
	{45, 0x82},
	{70, 0x6c},
	{100, 0x56},
	{120, 0x47},
	{140, 0x3a},
	{160, 0x2f},
	{180, 0x22},
	{200, 0x16},
	{220, 0xe},
	{240, 0x6},
	{260, 0x00},
};

void oplus_set_aod_dim_alpha(int cust)
{
	oplus_aod_dim_alpha = cust;
	DSI_DEBUG("set oplus_aod_dim_alpha = %d\n", oplus_aod_dim_alpha);
}

int oplus_get_panel_brightness(void)
{
	struct dsi_display *display = get_main_display();

	if (!display) {
		return 0;
	}

	return display->panel->bl_config.bl_level;
}

int oplus_get_panel_power_mode(void)
{
	struct dsi_display *display = get_main_display();

	if (!display)
		return -1;

	return display->panel->power_mode;
}

static int bl_to_alpha(int brightness)
{
	struct dsi_display *display = get_main_display();
	struct oplus_brightness_alpha *lut = NULL;
	int count = 0;
	int i = 0;
	int alpha;

	if (!display)
		return 0;

	if (display->panel->ba_seq && display->panel->ba_count) {
		count = display->panel->ba_count;
		lut = display->panel->ba_seq;
	} else {
		count = ARRAY_SIZE(brightness_alpha_lut);
		lut = brightness_alpha_lut;
	}

	for (i = 0; i < count; i++) {
		if (lut[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = lut[0].alpha;
	else if (i == count)
		alpha = lut[count - 1].alpha;
	else
		alpha = interpolate(brightness, lut[i-1].brightness,
				    lut[i].brightness, lut[i-1].alpha,
				    lut[i].alpha);

	return alpha;
}

static int bl_to_alpha_dc(int brightness)
{
	int level = ARRAY_SIZE(brightness_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut_dc); i++) {
		if (brightness_alpha_lut_dc[i].brightness >= brightness) {
			break;
		}
	}

	if (i == 0) {
		alpha = brightness_alpha_lut_dc[0].alpha;
	} else if (i == level) {
		alpha = brightness_alpha_lut_dc[level - 1].alpha;
	} else
		alpha = interpolate(brightness,
				brightness_alpha_lut_dc[i - 1].brightness,
				brightness_alpha_lut_dc[i].brightness,
				brightness_alpha_lut_dc[i - 1].alpha,
				brightness_alpha_lut_dc[i].alpha);

	return alpha;
}

static int brightness_to_alpha(int brightness)
{
	int alpha;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return 0;
	}

	if (brightness == 0 || brightness == 1) {
		if (!strcmp(display->panel->oplus_priv.vendor_name, "AMS643YE01")) {
			/*do nothing*/
		} else {
			brightness = oplus_last_backlight;
		}
	}

	if (oplus_dimlayer_hbm) {
		alpha = bl_to_alpha(brightness);

	} else {
		alpha = bl_to_alpha_dc(brightness);
	}

	return alpha;
}

static int oplus_get_panel_brightness_to_alpha(void)
{
	struct dsi_display *display = get_main_display();

	if (!display) {
		return 0;
	}

	if (oplus_panel_alpha) {
		return oplus_panel_alpha;
	}
	/* force dim layer alpha in AOD scene */
	if (oplus_aod_dim_alpha != CUST_A_NO) {
		if (oplus_aod_dim_alpha == CUST_A_TRANS)
			return 0;
		else if (oplus_aod_dim_alpha == CUST_A_OPAQUE)
			return 255;
	}
	if (hbm_mode) {
		return 0;
	}

	if (!oplus_ffl_trigger_finish) {
		return brightness_to_alpha(FFL_FP_LEVEL);
	}

	return brightness_to_alpha(display->panel->bl_config.bl_level);
}

int dsi_panel_parse_oplus_fod_config(struct dsi_panel *panel)
{
	int rc = 0;
	int i;
	u32 length = 0;
	u32 count = 0;
	u32 size = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;
	struct dsi_parser_utils *utils = &panel->utils;
	struct oplus_brightness_alpha *seq;

	arr = utils->get_property(utils->data, "oplus,dsi-fod-brightness", &length);

	if (!arr) {
		DSI_ERR("[%s] oplus,dsi-fod-brightness  not found\n", panel->name);
		return -EINVAL;
	}

	if (length & 0x1) {
		DSI_ERR("[%s] oplus,dsi-fod-brightness length error\n", panel->name);
		return -EINVAL;
	}

	DSI_DEBUG("RESET SEQ LENGTH = %d\n", length);
	length = length / sizeof(u32);
	size = length * sizeof(u32);

	arr_32 = kzalloc(size, GFP_KERNEL);

	if (!arr_32) {
		rc = -ENOMEM;
		goto error;
	}

	rc = utils->read_u32_array(utils->data, "oplus,dsi-fod-brightness",
			arr_32, length);

	if (rc) {
		DSI_ERR("[%s] cannot read dsi-fod-brightness\n", panel->name);
		goto error_free_arr_32;
	}

	count = length / 2;
	size = count * sizeof(*seq);
	seq = kzalloc(size, GFP_KERNEL);

	if (!seq) {
		rc = -ENOMEM;
		goto error_free_arr_32;
	}

	panel->ba_seq = seq;
	panel->ba_count = count;

	for (i = 0; i < length; i += 2) {
		seq->brightness = arr_32[i];
		seq->alpha = arr_32[i + 1];
		seq++;
	}

error_free_arr_32:
	kfree(arr_32);
error:
	return rc;
}

static int dsi_panel_parse_oplus_backlight_remapping_config(struct dsi_panel *panel)
{
	int rc = 0;
	int i;
	u32 length = 0;
	u32 count = 0;
	u32 size = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;
	struct dsi_parser_utils *utils = &panel->utils;
	struct oplus_brightness_alpha *bl_remap;

	panel->oplus_priv.bl_interpolate_nosub = utils->read_bool(utils->data,
			"oplus,bl_interpolate_nosub");

	arr = utils->get_property(utils->data, "oplus,dsi-brightness-remapping", &length);
	if (!arr) {
		DSI_DEBUG("[%s] oplus,dsi-brightness-remapping not found\n", panel->name);
		return -EINVAL;
	}

	if (length & 0x1) {
		DSI_ERR("[%s] oplus,dsi-brightness-remapping length error\n", panel->name);
		return -EINVAL;
	}

	DSI_INFO("oplus,dsi-brightness-remapping's length = %d, interpolate_nosub = %d\n", length, panel->oplus_priv.bl_interpolate_nosub ? 1 : 0);
	length = length / sizeof(u32);
	size = length * sizeof(u32);

	arr_32 = kzalloc(size, GFP_KERNEL);
	if (!arr_32) {
		rc = -ENOMEM;
		goto error;
	}

	rc = utils->read_u32_array(utils->data, "oplus,dsi-brightness-remapping",
			arr_32, length);
	if (rc) {
		DSI_ERR("[%s] cannot read oplus,dsi-brightness-remapping\n", panel->name);
		goto error_free_arr_32;
	}

	count = length / 2;
	size = count * sizeof(*bl_remap);
	bl_remap = kzalloc(size, GFP_KERNEL);
	if (!bl_remap) {
		rc = -ENOMEM;
		goto error_free_arr_32;
	}

	panel->oplus_priv.bl_remap = bl_remap;
	panel->oplus_priv.bl_remap_count = count;

	for (i = 0; i < length; i += 2) {
		bl_remap->brightness = arr_32[i];
		bl_remap->alpha = arr_32[i + 1];
		bl_remap++;
	}

error_free_arr_32:
	kfree(arr_32);
error:
	return rc;
}

int dsi_panel_parse_oplus_config(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = &panel->utils;

	dsi_panel_parse_oplus_fod_config(panel);
	dsi_panel_parse_oplus_backlight_remapping_config(panel);

	panel->oplus_priv.vendor_name = utils->get_property(utils->data,
			"oplus,mdss-dsi-vendor-name", NULL);

	if (!panel->oplus_priv.vendor_name) {
		pr_err("Failed to found panel name, using dumming name\n");
		panel->oplus_priv.vendor_name = DSI_PANEL_OPLUS_DUMMY_VENDOR_NAME;
	}

	panel->oplus_priv.manufacture_name = utils->get_property(utils->data,
			"oplus,mdss-dsi-manufacture", NULL);

	if (!panel->oplus_priv.manufacture_name) {
		pr_err("Failed to found panel name, using dumming name\n");
		panel->oplus_priv.manufacture_name = DSI_PANEL_OPLUS_DUMMY_MANUFACTURE_NAME;
	}

	panel->oplus_priv.is_pxlw_iris5 = utils->read_bool(utils->data,
			"oplus,is_pxlw_iris5");
	DSI_INFO("is_pxlw_iris5: %s",
		panel->oplus_priv.is_pxlw_iris5 ? "true" : "false");

#ifdef OPLUS_FEATURE_AOD_RAMLESS
	panel->oplus_priv.is_aod_ramless = utils->read_bool(utils->data,
			"oplus,aod_ramless");
	DSI_INFO("aod ramless mode: %s", panel->oplus_priv.is_aod_ramless ? "true" : "false");
#endif /* OPLUS_FEATURE_AOD_RAMLESS */

#ifdef OPLUS_BUG_STABILITY
	panel->oplus_priv.is_19781_lcd = utils->read_bool(utils->data,
			"oplus,is_19781_lcd");
	DSI_INFO("is_19781_lcd: %s",
		panel->oplus_priv.is_19781_lcd ? "true" : "false");

	 {
		u32 val = 0;
		int rc;
		rc = utils->read_u32(utils->data, "oplus,dsi-pll-delay", &val);
		panel->oplus_priv.pll_delay = !rc ? val : 0;
		DSI_INFO("oplus_dsi_pll_delay = %d\n", panel->oplus_priv.pll_delay);

		rc = utils->read_u32(utils->data, "oplus,prj-flag", &val);
		panel->oplus_priv.prj_flag = !rc ? val : 0;
		DSI_INFO("oplus_prj_flag = %d\n", panel->oplus_priv.prj_flag);
	}
#endif /* OPLUS_BUG_STABILITY */

	return 0;
}

int dsi_panel_parse_oplus_mode_config(struct dsi_display_mode *mode,
	struct dsi_parser_utils *utils)
{
	int rc;
	struct dsi_display_mode_priv_info *priv_info;
	int val = 0;

	priv_info = mode->priv_info;

	rc = utils->read_u32(utils->data, "oplus,fod-th-brightness", &val);

	if (rc) {
		DSI_ERR("oplus,fod-th-brightness is not defined, rc=%d\n", rc);
		priv_info->fod_th_brightness = 0;

	} else {
		priv_info->fod_th_brightness = val;
		DSI_INFO("oplus,fod-th-brightness is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-on-vblank", &val);

	if (rc) {
		DSI_ERR("oplus,fod-on-vblank is not defined, rc=%d\n", rc);
		priv_info->fod_on_vblank = 0;

	} else {
		priv_info->fod_on_vblank = val;
		DSI_INFO("oplus,fod-on-vblank is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-off-vblank", &val);

	if (rc) {
		DSI_ERR("oplus,fod-off-vblank is not defined, rc=%d\n", rc);
		priv_info->fod_off_vblank = 0;

	} else {
		priv_info->fod_off_vblank = val;
		DSI_INFO("oplus,fod-off-vblank is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-on-delay", &val);

	if (rc) {
		DSI_ERR("oplus,fod-on-delay is not defined, rc=%d\n", rc);
		priv_info->fod_on_delay = 0;

	} else {
		priv_info->fod_on_delay = val;
		DSI_INFO("oplus,fod-on-delay is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-off-delay", &val);

	if (rc) {
		DSI_ERR("oplus,fod-off-delay is not defined, rc=%d\n", rc);
		priv_info->fod_off_delay = 0;

	} else {
		priv_info->fod_off_delay = val;
		DSI_INFO("oplus,fod-off-delay is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-on-vblank_above_th", &val);

	if (rc) {
		DSI_ERR("oplus,fod-on-vblank_above_th is not defined, rc=%d\n", rc);
		priv_info->fod_on_vblank_above_th = 0;

	} else {
		priv_info->fod_on_vblank_above_th = val;
		DSI_INFO("oplus,fod-on-vblank_above_th is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-off-vblank_above_th", &val);

	if (rc) {
		DSI_ERR("oplus,fod-off-vblank_above_th is not defined, rc=%d\n", rc);
		priv_info->fod_off_vblank_above_th = 0;

	} else {
		priv_info->fod_off_vblank_above_th = val;
		DSI_INFO("oplus,fod-off-vblank_above_th is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-on-delay_above_th", &val);

	if (rc) {
		DSI_ERR("oplus,fod-on-delay_above_th is not defined, rc=%d\n", rc);
		priv_info->fod_on_delay_above_th = 0;

	} else {
		priv_info->fod_on_delay_above_th = val;
		DSI_INFO("oplus,fod-on-delay_above_th is %d", val);
	}

	rc = utils->read_u32(utils->data, "oplus,fod-off-delay_above_th", &val);

	if (rc) {
		DSI_ERR("oplus,fod-off-delay_above_th is not defined, rc=%d\n", rc);
		priv_info->fod_off_delay_above_th = 0;

	} else {
		priv_info->fod_off_delay_above_th = val;
		DSI_INFO("oplus,fod-off-delay_above_th is %d", val);
	}

	return 0;
}

bool sde_crtc_get_dimlayer_mode(struct drm_crtc_state *crtc_state)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state) {
		return false;
	}

	cstate = to_sde_crtc_state(crtc_state);
	return !!cstate->fingerprint_dim_layer;
}

bool sde_crtc_get_fingerprint_mode(struct drm_crtc_state *crtc_state)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state) {
		return false;
	}

	cstate = to_sde_crtc_state(crtc_state);
	return !!cstate->fingerprint_mode;
}

bool sde_crtc_get_fingerprint_pressed(struct drm_crtc_state *crtc_state)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state) {
		return false;
	}

	cstate = to_sde_crtc_state(crtc_state);
	return cstate->fingerprint_pressed;
}

bool sde_connector_get_fp_mode(struct drm_connector *connector)
{
	if (!connector || !connector->state || !connector->state->crtc)
		return false;

	return sde_crtc_get_fingerprint_mode(connector->state->crtc->state);
}

bool sde_connector_get_fppress_mode(struct drm_connector *connector)
{
	if (!connector || !connector->state || !connector->state->crtc)
		return false;

	return sde_crtc_get_fingerprint_pressed(connector->state->crtc->state);
}

int sde_crtc_set_onscreenfinger_defer_sync(struct drm_crtc_state *crtc_state,
	bool defer_sync)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state) {
		return -EINVAL;
	}

	cstate = to_sde_crtc_state(crtc_state);
	cstate->fingerprint_defer_sync = defer_sync;
	return 0;
}

int sde_crtc_config_fingerprint_dim_layer(struct drm_crtc_state *crtc_state,
	int stage)
{
	struct sde_crtc_state *cstate;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct sde_hw_dim_layer *fingerprint_dim_layer;
	int alpha = oplus_get_panel_brightness_to_alpha();
	struct sde_kms *kms;

	kms = _sde_crtc_get_kms_(crtc_state->crtc);

	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return -EINVAL;
	}

	cstate = to_sde_crtc_state(crtc_state);

	if (cstate->num_dim_layers == SDE_MAX_DIM_LAYERS - 1) {
		pr_err("failed to get available dim layer for custom\n");
		return -EINVAL;
	}

	if ((stage + SDE_STAGE_0) >= kms->catalog->mixer[0].sblk->maxblendstages) {
		return -EINVAL;
	}

	fingerprint_dim_layer = &cstate->dim_layer[cstate->num_dim_layers];
	fingerprint_dim_layer->flags = SDE_DRM_DIM_LAYER_INCLUSIVE;
	fingerprint_dim_layer->stage = stage + SDE_STAGE_0;

	DSI_DEBUG("fingerprint_dim_layer: stage = %d, alpha = %d\n", stage, alpha);

	fingerprint_dim_layer->rect.x = 0;
	fingerprint_dim_layer->rect.y = 0;
	fingerprint_dim_layer->rect.w = mode->hdisplay;
	fingerprint_dim_layer->rect.h = mode->vdisplay;
	fingerprint_dim_layer->color_fill = (struct sde_mdss_color) {
		0, 0, 0, alpha
	};
	cstate->fingerprint_dim_layer = fingerprint_dim_layer;
	oplus_underbrightness_alpha = alpha;

	return 0;
}

bool is_skip_pcc(struct drm_crtc *crtc)
{
	if (OPLUS_DISPLAY_POWER_DOZE_SUSPEND == get_oplus_display_power_status() ||
		OPLUS_DISPLAY_POWER_DOZE == get_oplus_display_power_status() ||
		sde_crtc_get_fingerprint_mode(crtc->state)) {
		return true;
	}

	return false;
}

bool sde_cp_crtc_update_pcc(struct drm_crtc *crtc)
{
	struct sde_hw_cp_cfg hw_cfg;
	struct sde_hw_dspp *hw_dspp;
	struct sde_hw_mixer *hw_lm;
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_mdss_cfg *catalog = NULL;
	u32 num_mixers = sde_crtc->num_mixers;
	bool pcc_skip_mode;
	int i = 0;

	if (!is_dsi_panel(&sde_crtc->base)) {
		return false;
	}

	pcc_skip_mode = is_skip_pcc(crtc);
	if (oplus_skip_pcc == pcc_skip_mode)
		return false;

	oplus_skip_pcc = pcc_skip_mode;
	memset(&hw_cfg, 0, sizeof(hw_cfg));

	if (!pcc_skip_mode && oplus_pcc_enabled) {
		hw_cfg.payload = &oplus_save_pcc;
		hw_cfg.len = sizeof(oplus_save_pcc);
	}

	hw_cfg.num_of_mixers = sde_crtc->num_mixers;
	hw_cfg.last_feature = 0;

	for (i = 0; i < num_mixers; i++) {
		hw_dspp = sde_crtc->mixers[i].hw_dspp;
		if (!hw_dspp || i >= DSPP_MAX)
			continue;
		hw_cfg.dspp[i] = hw_dspp;
	}

	catalog = get_kms_(&sde_crtc->base)->catalog;
	hw_cfg.broadcast_disabled = catalog->dma_cfg.broadcast_disabled;
	for (i = 0; i < num_mixers; i++) {
		hw_lm = sde_crtc->mixers[i].hw_lm;
		hw_dspp = sde_crtc->mixers[i].hw_dspp;

		if (!hw_lm) {
			continue;
		}

		if (!hw_dspp || !hw_dspp->ops.setup_pcc) {
			continue;
		}

		hw_cfg.ctl = sde_crtc->mixers[i].hw_ctl;
		hw_cfg.mixer_info = hw_lm;
		hw_cfg.displayh = num_mixers * hw_lm->cfg.out_width;
		hw_cfg.displayv = hw_lm->cfg.out_height;
		hw_dspp->ops.setup_pcc(hw_dspp, &hw_cfg);
	}

	return true;
}

bool
_sde_encoder_setup_dither_for_onscreenfingerprint(
			struct sde_encoder_phys *phys, void *dither_cfg, int len)
{
	struct drm_encoder *drm_enc = phys->parent;
	struct drm_msm_dither dither;

	if (!drm_enc || !drm_enc->crtc)
		return -EFAULT;

	if (!sde_crtc_get_dimlayer_mode(drm_enc->crtc->state))
		return -EINVAL;

	if (len != sizeof(dither))
		return -EINVAL;

	if (oplus_get_panel_brightness_to_alpha() < oplus_dimlayer_dither_threshold)
		return -EINVAL;

	memcpy(&dither, dither_cfg, len);
	dither.c0_bitdepth = 6;
	dither.c1_bitdepth = 8;
	dither.c2_bitdepth = 8;
	dither.c3_bitdepth = 8;
	dither.temporal_en = 1;

	phys->hw_pp->ops.setup_dither(phys->hw_pp, &dither, len);

	return 0;
}
int sde_plane_check_fingerprint_layer(const struct drm_plane_state *drm_state)
{
	struct sde_plane_state *pstate;

	if (!drm_state) {
		return 0;
	}

	pstate = to_sde_plane_state(drm_state);

	return sde_plane_get_property(pstate, PLANE_PROP_CUSTOM);
}

int oplus_display_panel_get_dimlayer_hbm(void *data)
{
	uint32_t *dimlayer_hbm = data;

	(*dimlayer_hbm) = oplus_dimlayer_hbm;

	return 0;
}

