subdir-ccflags-y := -I$(srctree)/drivers/net/ethernet/mellanox/mlx5/
obj-$(CONFIG_MLX5_CORE)		+= mlx5_core.o

mlx5_core-y := main.o pagealloc.o health.o \
	       cmd.o debugfs.o eq.o lag.o dev.o

mlx5_core-y += lib/fw.o lib/uar.o lib/mcg.o lib/cq.o \
	       lib/srq.o lib/alloc.o lib/qp.o lib/port.o lib/mr.o \
	       lib/pd.o lib/mad.o lib/transobj.o lib/vport.o lib/rl.o

mlx5_core-y += sriov/sriov.o
mlx5_core-y += fs/cmd.o fs/core.o fs/counters.o

mlx5_core-$(CONFIG_MLX5_CORE_EN) += en/wq.o en/main.o en/common.o \
		en/fs.o en/ethtool.o en/tx.o en/rx.o en/rx_am.o \
		en/txrx.o en/clock.o en/vxlan.o en/tc.o en/arfs.o \
		en/rep.o en/fs_ethtool.o en/selftest.o

mlx5_core-$(CONFIG_MLX5_CORE_EN) += sriov/eswitch.o sriov/eswitch_offloads.o

mlx5_core-$(CONFIG_MLX5_CORE_EN_DCB) += en/dcbnl.o
