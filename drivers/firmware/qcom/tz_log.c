/* Copyright (c) 2011-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/msm_ion.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of.h>

#include <soc/qcom/scm.h>
#include <soc/qcom/qseecomi.h>

/* QSEE_LOG_BUF_SIZE = 32K */
#define QSEE_LOG_BUF_SIZE 0x4000

/* TZ Diagnostic Area legacy version number */
#define TZBSP_DIAG_MAJOR_VERSION_LEGACY	2
/*
 * Preprocessor Definitions and Constants
 */
#define TZBSP_MAX_CPU_COUNT 0x08
/*
 * Number of VMID Tables
 */
#define TZBSP_DIAG_NUM_OF_VMID 16
/*
 * VMID Description length
 */
#define TZBSP_DIAG_VMID_DESC_LEN 7
/*
 * Number of Interrupts
 */
#define TZBSP_DIAG_INT_NUM  32
/*
 * Length of descriptive name associated with Interrupt
 */
#define TZBSP_MAX_INT_DESC 16
/*
 * TZ 3.X version info
 */
#define QSEE_VERSION_TZ_3_X 0x800000
/*
 * TZ 4.X version info
 */
#define QSEE_VERSION_TZ_4_X 0x1000000

#define TZBSP_AES_256_ENCRYPTED_KEY_SIZE 256
#define TZBSP_NONCE_LEN 12
#define TZBSP_TAG_LEN 16

/*
 * VMID Table
 */
struct tzdbg_vmid_t {
	uint8_t vmid; /* Virtual Machine Identifier */
	uint8_t desc[TZBSP_DIAG_VMID_DESC_LEN];	/* ASCII Text */
};
/*
 * Boot Info Table
 */
struct tzdbg_boot_info_t {
	uint32_t wb_entry_cnt;	/* Warmboot entry CPU Counter */
	uint32_t wb_exit_cnt;	/* Warmboot exit CPU Counter */
	uint32_t pc_entry_cnt;	/* Power Collapse entry CPU Counter */
	uint32_t pc_exit_cnt;	/* Power Collapse exit CPU counter */
	uint32_t warm_jmp_addr;	/* Last Warmboot Jump Address */
	uint32_t spare;	/* Reserved for future use. */
};
/*
 * Boot Info Table for 64-bit
 */
struct tzdbg_boot_info64_t {
	uint32_t wb_entry_cnt;  /* Warmboot entry CPU Counter */
	uint32_t wb_exit_cnt;   /* Warmboot exit CPU Counter */
	uint32_t pc_entry_cnt;  /* Power Collapse entry CPU Counter */
	uint32_t pc_exit_cnt;   /* Power Collapse exit CPU counter */
	uint32_t psci_entry_cnt;/* PSCI syscall entry CPU Counter */
	uint32_t psci_exit_cnt;   /* PSCI syscall exit CPU Counter */
	uint64_t warm_jmp_addr; /* Last Warmboot Jump Address */
	uint32_t warm_jmp_instr; /* Last Warmboot Jump Address Instruction */
};
/*
 * Reset Info Table
 */
struct tzdbg_reset_info_t {
	uint32_t reset_type;	/* Reset Reason */
	uint32_t reset_cnt;	/* Number of resets occurred/CPU */
};
/*
 * Interrupt Info Table
 */
struct tzdbg_int_t {
	/*
	 * Type of Interrupt/exception
	 */
	uint16_t int_info;
	/*
	 * Availability of the slot
	 */
	uint8_t avail;
	/*
	 * Reserved for future use
	 */
	uint8_t spare;
	/*
	 * Interrupt # for IRQ and FIQ
	 */
	uint32_t int_num;
	/*
	 * ASCII text describing type of interrupt e.g:
	 * Secure Timer, EBI XPU. This string is always null terminated,
	 * supporting at most TZBSP_MAX_INT_DESC characters.
	 * Any additional characters are truncated.
	 */
	uint8_t int_desc[TZBSP_MAX_INT_DESC];
	uint64_t int_count[TZBSP_MAX_CPU_COUNT]; /* # of times seen per CPU */
};

/*
 * Interrupt Info Table used in tz version >=4.X
 */
struct tzdbg_int_t_tz40 {
	uint16_t int_info;
	uint8_t avail;
	uint8_t spare;
	uint32_t int_num;
	uint8_t int_desc[TZBSP_MAX_INT_DESC];
	uint32_t int_count[TZBSP_MAX_CPU_COUNT]; /* uint32_t in TZ ver >= 4.x*/
};

/* warm boot reason for cores */
struct tzbsp_diag_wakeup_info_t {
	/* Wake source info : APCS_GICC_HPPIR */
	uint32_t HPPIR;
	/* Wake source info : APCS_GICC_AHPPIR */
	uint32_t AHPPIR;
};

/*
 * Log ring buffer position
 */
struct tzdbg_log_pos_t {
	uint16_t wrap;
	uint16_t offset;
};

 /*
  * Log ring buffer
  */
struct tzdbg_log_t {
	struct tzdbg_log_pos_t	log_pos;
	/* open ended array to the end of the 4K IMEM buffer */
	uint8_t					log_buf[];
};

/*
 * Diagnostic Table
 * Note: This is the reference data structure for tz diagnostic table
 * supporting TZBSP_MAX_CPU_COUNT, the real diagnostic data is directly
 * copied into buffer from i/o memory.
 */
struct tzdbg_t {
	uint32_t magic_num;
	uint32_t version;
	/*
	 * Number of CPU's
	 */
	uint32_t cpu_count;
	/*
	 * Offset of VMID Table
	 */
	uint32_t vmid_info_off;
	/*
	 * Offset of Boot Table
	 */
	uint32_t boot_info_off;
	/*
	 * Offset of Reset info Table
	 */
	uint32_t reset_info_off;
	/*
	 * Offset of Interrupt info Table
	 */
	uint32_t int_info_off;
	/*
	 * Ring Buffer Offset
	 */
	uint32_t ring_off;
	/*
	 * Ring Buffer Length
	 */
	uint32_t ring_len;

	/* Offset for Wakeup info */
	uint32_t wakeup_info_off;

	/*
	 * VMID to EE Mapping
	 */
	struct tzdbg_vmid_t vmid_info[TZBSP_DIAG_NUM_OF_VMID];
	/*
	 * Boot Info
	 */
	struct tzdbg_boot_info_t  boot_info[TZBSP_MAX_CPU_COUNT];
	/*
	 * Reset Info
	 */
	struct tzdbg_reset_info_t reset_info[TZBSP_MAX_CPU_COUNT];
	uint32_t num_interrupts;
	struct tzdbg_int_t  int_info[TZBSP_DIAG_INT_NUM];

	/* Wake up info */
	struct tzbsp_diag_wakeup_info_t  wakeup_info[TZBSP_MAX_CPU_COUNT];

	uint8_t key[TZBSP_AES_256_ENCRYPTED_KEY_SIZE];

	uint8_t nonce[TZBSP_NONCE_LEN];

	uint8_t tag[TZBSP_TAG_LEN];

	/*
	 * We need at least 2K for the ring buffer
	 */
	struct tzdbg_log_t ring_buffer;	/* TZ Ring Buffer */
};

struct hypdbg_log_pos_t {
	uint16_t wrap;
	uint16_t offset;
};

struct hypdbg_boot_info_t {
	uint32_t warm_entry_cnt;
	uint32_t warm_exit_cnt;
};

struct hypdbg_t {
	/* Magic Number */
	uint32_t magic_num;

	/* Number of CPU's */
	uint32_t cpu_count;

	/* Ring Buffer Offset */
	uint32_t ring_off;

	/* Ring buffer position mgmt */
	struct hypdbg_log_pos_t log_pos;
	uint32_t log_len;

	/* S2 fault numbers */
	uint32_t s2_fault_counter;

	/* Boot Info */
	struct hypdbg_boot_info_t boot_info[TZBSP_MAX_CPU_COUNT];

	/* Ring buffer pointer */
	uint8_t log_buf_p[];
};

/*
 * Enumeration order for VMID's
 */
enum tzdbg_stats_type {
	TZDBG_BOOT = 0,
	TZDBG_RESET,
	TZDBG_INTERRUPT,
	TZDBG_VMID,
	TZDBG_GENERAL,
	TZDBG_LOG,
	TZDBG_QSEE_LOG,
	TZDBG_HYP_GENERAL,
	TZDBG_HYP_LOG,
	TZDBG_STATS_MAX
};

struct tzdbg_stat {
	char *name;
	char *data;
};

struct tzdbg {
	void __iomem *virt_iobase;
	void __iomem *hyp_virt_iobase;
	struct tzdbg_t *diag_buf;
	struct hypdbg_t *hyp_diag_buf;
	char *disp_buf;
	int debug_tz[TZDBG_STATS_MAX];
	struct tzdbg_stat stat[TZDBG_STATS_MAX];
	uint32_t hyp_debug_rw_buf_size;
	bool is_hyplog_enabled;
	uint32_t tz_version;
};

static struct tzdbg tzdbg = {
	.stat[TZDBG_BOOT].name = "boot",
	.stat[TZDBG_RESET].name = "reset",
	.stat[TZDBG_INTERRUPT].name = "interrupt",
	.stat[TZDBG_VMID].name = "vmid",
	.stat[TZDBG_GENERAL].name = "general",
	.stat[TZDBG_LOG].name = "log",
	.stat[TZDBG_QSEE_LOG].name = "qsee_log",
	.stat[TZDBG_HYP_GENERAL].name = "hyp_general",
	.stat[TZDBG_HYP_LOG].name = "hyp_log",
};

static struct tzdbg_log_t *g_qsee_log;
static uint32_t debug_rw_buf_size;

static struct ion_client  *g_ion_clnt;
static struct ion_handle *g_ihandle;

/*
 * Allocates log buffer from ION, registers the buffer at TZ
 */
static void tzdbg_register_qsee_log_buf(void)
{
	/* register log buffer scm request */
	struct qseecom_reg_log_buf_ireq req;

	/* scm response */
	struct qseecom_command_scm_resp resp = {};
	ion_phys_addr_t pa = 0;
	size_t len;
	int ret = 0;

	/* Create ION msm client */
	g_ion_clnt = msm_ion_client_create("qsee_log");
	if (g_ion_clnt == NULL) {
		pr_err("%s: Ion client cannot be created\n", __func__);
		return;
	}

	g_ihandle = ion_alloc(g_ion_clnt, QSEE_LOG_BUF_SIZE,
			4096, ION_HEAP(ION_QSECOM_HEAP_ID), 0);
	if (IS_ERR_OR_NULL(g_ihandle)) {
		pr_err("%s: Ion client could not retrieve the handle\n",
			__func__);
		goto err1;
	}

	ret = ion_phys(g_ion_clnt, g_ihandle, &pa, &len);
	if (ret) {
		pr_err("%s: Ion conversion to physical address failed\n",
			__func__);
		goto err2;
	}

	req.qsee_cmd_id = QSEOS_REGISTER_LOG_BUF_COMMAND;
	req.phy_addr = (uint32_t)pa;
	req.len = len;

	if (!is_scm_armv8()) {
		/*  SCM_CALL  to register the log buffer */
		ret = scm_call(SCM_SVC_TZSCHEDULER, 1,  &req, sizeof(req),
			&resp, sizeof(resp));
	} else {
		struct scm_desc desc = {0};

		desc.args[0] = pa;
		desc.args[1] = len;
		desc.arginfo = 0x22;
		ret = scm_call2(SCM_QSEEOS_FNID(1, 6), &desc);
		resp.result = desc.ret[0];
	}

	if (ret) {
		pr_err("%s: scm_call to register log buffer failed\n",
			__func__);
		goto err2;
	}

	if (resp.result != QSEOS_RESULT_SUCCESS) {
		pr_err(
		"%s: scm_call to register log buf failed, resp result =%d\n",
		__func__, resp.result);
		goto err2;
	}

	g_qsee_log =
		(struct tzdbg_log_t *)ion_map_kernel(g_ion_clnt, g_ihandle);

	if (IS_ERR(g_qsee_log)) {
		pr_err("%s: Couldn't map ion buffer to kernel\n",
			__func__);
		goto err2;
	}

	g_qsee_log->log_pos.wrap = g_qsee_log->log_pos.offset = 0;
	return;

err2:
	ion_free(g_ion_clnt, g_ihandle);
	g_ihandle = NULL;
err1:
	ion_client_destroy(g_ion_clnt);
	g_ion_clnt = NULL;
}

static int  tzdbgfs_init(struct platform_device *pdev)
{
	int rc = 0;

	return rc;
}

static void tzdbgfs_exit(struct platform_device *pdev)
{
	if (g_ion_clnt != NULL) {
		if (!IS_ERR_OR_NULL(g_ihandle)) {
			ion_unmap_kernel(g_ion_clnt, g_ihandle);
			ion_free(g_ion_clnt, g_ihandle);
		}
		ion_client_destroy(g_ion_clnt);
	}
}

static int __update_hypdbg_base(struct platform_device *pdev,
			void __iomem *virt_iobase)
{
	phys_addr_t hypdiag_phy_iobase;
	uint32_t hyp_address_offset;
	uint32_t hyp_size_offset;
	struct hypdbg_t *hyp;
	uint32_t *ptr = NULL;

	if (of_property_read_u32((&pdev->dev)->of_node, "hyplog-address-offset",
							&hyp_address_offset)) {
		dev_err(&pdev->dev, "hyplog address offset is not defined\n");
		return -EINVAL;
	}
	if (of_property_read_u32((&pdev->dev)->of_node, "hyplog-size-offset",
							&hyp_size_offset)) {
		dev_err(&pdev->dev, "hyplog size offset is not defined\n");
		return -EINVAL;
	}

	hypdiag_phy_iobase = readl_relaxed(virt_iobase + hyp_address_offset);
	tzdbg.hyp_debug_rw_buf_size = readl_relaxed(virt_iobase +
					hyp_size_offset);

	tzdbg.hyp_virt_iobase = devm_ioremap_nocache(&pdev->dev,
					hypdiag_phy_iobase,
					tzdbg.hyp_debug_rw_buf_size);
	if (!tzdbg.hyp_virt_iobase) {
		dev_err(&pdev->dev, "ERROR could not ioremap: start=%pr, len=%u\n",
			&hypdiag_phy_iobase, tzdbg.hyp_debug_rw_buf_size);
		return -ENXIO;
	}

	ptr = kzalloc(tzdbg.hyp_debug_rw_buf_size, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	tzdbg.hyp_diag_buf = (struct hypdbg_t *)ptr;
	hyp = tzdbg.hyp_diag_buf;
	hyp->log_pos.wrap = hyp->log_pos.offset = 0;
	return 0;
}

static void tzdbg_get_tz_version(void)
{
	uint32_t smc_id = 0;
	uint32_t feature = 10;
	struct qseecom_command_scm_resp resp = {0};
	struct scm_desc desc = {0};
	int ret = 0;

	if (!is_scm_armv8()) {
		ret = scm_call(SCM_SVC_INFO, SCM_SVC_UTIL,  &feature,
					sizeof(feature), &resp, sizeof(resp));
	} else {
		smc_id = TZ_INFO_GET_FEATURE_VERSION_ID;
		desc.arginfo = TZ_INFO_GET_FEATURE_VERSION_ID_PARAM_ID;
		desc.args[0] = feature;
		ret = scm_call2(smc_id, &desc);
		resp.result = desc.ret[0];
	}

	if (ret)
		pr_err("%s: scm_call to get tz version failed\n",
				__func__);
	else
		tzdbg.tz_version = resp.result;

}

/*
 * Driver functions
 */
static int tz_log_probe(struct platform_device *pdev)
{
	struct resource *resource;
	void __iomem *virt_iobase;
	phys_addr_t tzdiag_phy_iobase;
	uint32_t *ptr = NULL;
	int ret = 0;

	/*
	 * Get address that stores the physical location diagnostic data
	 */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&pdev->dev,
				"%s: ERROR Missing MEM resource\n", __func__);
		return -ENXIO;
	};

	/*
	 * Get the debug buffer size
	 */
	debug_rw_buf_size = resource->end - resource->start + 1;

	/*
	 * Map address that stores the physical location diagnostic data
	 */
	virt_iobase = devm_ioremap_nocache(&pdev->dev, resource->start,
				debug_rw_buf_size);
	if (!virt_iobase) {
		dev_err(&pdev->dev,
			"%s: ERROR could not ioremap: start=%pr, len=%u\n",
			__func__, &resource->start,
			(unsigned int)(debug_rw_buf_size));
		return -ENXIO;
	}

	if (pdev->dev.of_node) {
		tzdbg.is_hyplog_enabled = of_property_read_bool(
			(&pdev->dev)->of_node, "qcom,hyplog-enabled");
		if (tzdbg.is_hyplog_enabled) {
			ret = __update_hypdbg_base(pdev, virt_iobase);
			if (ret) {
				dev_err(&pdev->dev, "%s() failed to get device tree data ret = %d\n",
						__func__, ret);
				return -EINVAL;
			}
		} else {
			dev_info(&pdev->dev, "Hyp log service is not supported\n");
		}
	} else {
		dev_dbg(&pdev->dev, "Device tree data is not found\n");
	}

	/*
	 * Retrieve the address of diagnostic data
	 */
	tzdiag_phy_iobase = readl_relaxed(virt_iobase);

	/*
	 * Map the diagnostic information area
	 */
	tzdbg.virt_iobase = devm_ioremap_nocache(&pdev->dev,
				tzdiag_phy_iobase, debug_rw_buf_size);

	if (!tzdbg.virt_iobase) {
		dev_err(&pdev->dev,
			"%s: ERROR could not ioremap: start=%pr, len=%u\n",
			__func__, &tzdiag_phy_iobase,
			debug_rw_buf_size);
		return -ENXIO;
	}

	ptr = kzalloc(debug_rw_buf_size, GFP_KERNEL);
	if (ptr == NULL)
		return -ENXIO;

	tzdbg.diag_buf = (struct tzdbg_t *)ptr;

	if (tzdbgfs_init(pdev)) {
		kfree(tzdbg.diag_buf);
		tzdbg.diag_buf = NULL;
	}

	tzdbg_register_qsee_log_buf();

	tzdbg_get_tz_version();

	return 0;
}


static int tz_log_remove(struct platform_device *pdev)
{
	kzfree(tzdbg.diag_buf);
	if (tzdbg.hyp_diag_buf)
		kzfree(tzdbg.hyp_diag_buf);
	tzdbgfs_exit(pdev);

	return 0;
}

static const struct of_device_id tzlog_match[] = {
	{	.compatible = "qcom,tz-log",
	},
	{}
};

static struct platform_driver tz_log_driver = {
	.probe		= tz_log_probe,
	.remove		= tz_log_remove,
	.driver		= {
		.name = "tz_log",
		.owner = THIS_MODULE,
		.of_match_table = tzlog_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

static int __init tz_log_init(void)
{
	return platform_driver_register(&tz_log_driver);
}

static void __exit tz_log_exit(void)
{
	platform_driver_unregister(&tz_log_driver);
}

module_init(tz_log_init);
module_exit(tz_log_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TZ Log driver");
MODULE_ALIAS("platform:tz_log");
