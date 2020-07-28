#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel_stat.h>
#include <linux/hwinfo.h>
#include <linux/cpu.h>
#include <soc/qcom/socinfo.h>
#include <soc/qcom/smem.h>

#define HWINFO_DDRID_SAMSUNG	0x01
#define HWINFO_DDRID_HYNIX	0x06
#define HWINFO_DDRID_ELPIDA	0x03
#define HWINFO_DDRID_MICRON	0xFF
#define HWINFO_DDRID_NANYA	0x05
#define HWINFO_DDRID_INTEL	0x0E

struct {
	unsigned int touch_info:4;
	unsigned int soc_info:4;
	unsigned int ddr_info:4;
	unsigned int emmc_info:16;
	unsigned int cpu_info:2;
	unsigned int pmic_info:2;
	unsigned int panel_info:4;
	unsigned int tp_maker_info:6;
	unsigned int fp_info:1;
} hw_info;

static int hwinfo_proc_show(struct seq_file *m, void *v)
{
	switch (hw_info.emmc_info) {
	case 0x0198:
		seq_printf(m, "UFS: Toshiba");
		break;
	case 0x01ce:
		seq_printf(m, "UFS: Samsung");
		break;
	case 0x01ad:
		seq_printf(m, "UFS: Hynix");
		break;
	}

	switch (hw_info.ddr_info) {
	case HWINFO_DDRID_SAMSUNG:
		seq_printf(m, "DDR: Samsung");
		break;
	case HWINFO_DDRID_HYNIX:
		seq_printf(m, "DDR: Hynix");
		break;
	case HWINFO_DDRID_ELPIDA:
		seq_printf(m, "DDR: Elpida");
		break;
	case HWINFO_DDRID_NANYA:
		seq_printf(m, "DDR: Nanya");
		break;
	}

	switch (hw_info.touch_info) {
	case 1:
		seq_printf(m, "Touch: Synaptics");
		break;
	case 2:
		seq_printf(m, "Touch: Philips");
		break;
	case 3:
		seq_printf(m, "Touch: Fts");
		break;
	}

	switch (hw_info.tp_maker_info) {
	case 1:
		seq_printf(m, "TP: Biel");
		break;
	case 2:
		seq_printf(m, "TP: Lens");
		break;
	case 4:
		seq_printf(m, "TP: Ofilm");
		break;
	case 8:
		seq_printf(m, "TP: Sharp");
		break;
	case 17:
		seq_printf(m, "TP: Ebbg");
		break;
	case 18:
		seq_printf(m, "TP: LG");
		break;
	case 22:
		seq_printf(m, "TP: Tianma");
		break;
	case 24:
		seq_printf(m, "TP: Sdc");
		break;
	case 33:
		seq_printf(m, "TP: N");
		break;
	}

	switch (hw_info.panel_info) {
	case 0:
		seq_printf(m, "LCD: JDI Panel");
		break;
	case 4:
		seq_printf(m, "LCD: LG Panel");
		break;
	}

	switch (hw_info.fp_info) {
	case 0:
		seq_printf(m, "Fingerprint: Goodix");
		break;
	case 1:
		seq_printf(m, "Fingerprint: FPC");
		break;
	}

	return 0;
}


void update_hardware_info(unsigned int type, unsigned int value)
{
	switch (type) {
	case TYPE_TOUCH:
		hw_info.touch_info = value;
		break;
	case TYPE_SOC:
		hw_info.soc_info = value;
		break;
	case TYPE_DDR:
		hw_info.ddr_info = value;
		break;
	case TYPE_EMMC:
		hw_info.emmc_info = value;
		break;
	case TYPE_CPU:
		hw_info.cpu_info = value;
		break;
	case TYPE_PMIC:
		hw_info.pmic_info = value;
		break;
	case TYPE_PANEL:
		hw_info.panel_info = value;
		break;
	case TYPE_TP_MAKER:
		hw_info.tp_maker_info = value;
		break;
	case TYPE_FP:
		hw_info.fp_info = value;
		break;
	default:
		break;
	}
}

unsigned int get_hardware_info(unsigned int type)
{
	unsigned int ret = 0xFF;

	switch (type) {
	case TYPE_TOUCH:
		ret = hw_info.touch_info;
		break;
	case TYPE_SOC:
		ret = hw_info.soc_info;
		break;
	case TYPE_DDR:
		ret = hw_info.ddr_info;
		break;
	case TYPE_EMMC:
		ret = hw_info.emmc_info;
		break;
	case TYPE_CPU:
		ret = hw_info.cpu_info;
		break;
	case TYPE_PMIC:
		ret = hw_info.pmic_info;
		break;
	case TYPE_PANEL:
		ret = hw_info.panel_info;
		break;
	case TYPE_TP_MAKER:
		ret = hw_info.tp_maker_info;
		break;
	case TYPE_FP:
		ret = hw_info.fp_info;
		break;
	default:
		break;
	}

	return ret;
}

static int hwinfo_get_ddr_info_from_smem(void)
{
	unsigned size;
	uint32_t *ddr_table_ptr;

	ddr_table_ptr = smem_get_entry(SMEM_ID_VENDOR2, &size, 0,
			SMEM_ANY_HOST_FLAG);
	if (IS_ERR(ddr_table_ptr)) {
		hw_info.ddr_info = 0;
		return PTR_ERR(ddr_table_ptr);
	}

	hw_info.ddr_info = *ddr_table_ptr;
	return 0;
}

static int hwinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwinfo_proc_show, NULL);
}

static const struct file_operations hwinfo_proc_fops = {
	.open		= hwinfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int cpumaxfreq_show(struct seq_file *m, void *v)
{
	seq_printf(m, "2.83");

	return 0;
}

static int cpumaxfreq_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpumaxfreq_show, NULL);
}

static const struct file_operations proc_cpumaxfreq_operations = {
	.open       = cpumaxfreq_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static int __init proc_hwinfo_init(void)
{
	hwinfo_get_ddr_info_from_smem();
	proc_create("cpumaxfreq", 0, NULL, &proc_cpumaxfreq_operations);
	proc_create("hwinfo", 0, NULL, &hwinfo_proc_fops);
	return 0;
}
late_initcall(proc_hwinfo_init);