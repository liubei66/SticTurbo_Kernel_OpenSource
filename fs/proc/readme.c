#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int readme_show(struct seq_file *m, void *v)
{
	/* 小米6/小米MIX 2库存 */
	seq_printf(m, "readme-test\n");

	return 0;
}

static int readme_open(struct inode *inode, struct file *file)
{
	return single_open(file, readme_show, NULL);
}

static const struct file_operations proc_readme_operations = {
	.open       = readme_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static int __init proc_readme_init(void)
{
	proc_create("readme", 0, NULL, &proc_readme_operations);
	return 0;
}
fs_initcall(proc_readme_init);
