# Check open filename arg using perf trace + vfs_getname

# Uses the 'perf test shell' library to add probe:vfs_getname to the system
# then use it with 'perf trace' using 'touch' to write to a temp file, then
# checks that that was captured by the vfs_getname was used by 'perf trace',
# that already handles "probe:vfs_getname" if present, and used in the
# "open" syscall "filename" argument beautifier.

# Arnaldo Carvalho de Melo <acme@kernel.org>, 2017

. $(dirname $0)/lib/probe.sh

skip_if_no_perf_probe || exit 2

. $(dirname $0)/lib/probe_vfs_getname.sh

file=$(mktemp /tmp/temporary_file.XXXXX)

trace_open_vfs_getname() {
	test "$(uname -m)" = s390x && { svc="openat"; txt="dfd: +CWD, +"; }

	perf trace -e ${svc:-open} touch $file 2>&1 | \
	egrep " +[0-9]+\.[0-9]+ +\( +[0-9]+\.[0-9]+ ms\): +touch\/[0-9]+ ${svc:-open}\(${txt}filename: +${file}, +flags: CREAT\|NOCTTY\|NONBLOCK\|WRONLY, +mode: +IRUGO\|IWUGO\) += +[0-9]+$"
}


add_probe_vfs_getname || skip_if_no_debuginfo
err=$?
if [ $err -ne 0 ] ; then
	exit $err
fi

# Do not use whatever ~/.perfconfig file, it may change the output
# via trace.{show_timestamp,show_prefix,etc}
export PERF_CONFIG=/dev/null

trace_open_vfs_getname
err=$?
rm -f ${file}
cleanup_probe_vfs_getname
exit $err
