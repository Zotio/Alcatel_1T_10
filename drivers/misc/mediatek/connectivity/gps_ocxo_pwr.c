/*
*******************************************
*******************************************
*******************************************
*******************************************
				gps vcn2.8v  driver
*******************************************
*******************************************
*******************************************
*******************************************
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif

#include <linux/regulator/consumer.h>

#ifdef CONFIG_OF
static const struct of_device_id ocxo_pwr_of_match[] = {
        {.compatible = "mediatek,mt6580-ocxo"},
        {},
};
MODULE_DEVICE_TABLE(of, ocxo_pwr_of_match);

/* get LDO supply */
static int gps_get_vcn_supply(struct device *dev)
{
	int ret;
	unsigned int volt;
	struct regulator *gps_vcn;
	pr_debug("gps: gps_get_vcn_supply is going\n");

	gps_vcn = devm_regulator_get(dev, "reg-gps-vcn");
	if (IS_ERR(gps_vcn)) {
		ret = PTR_ERR(gps_vcn);
		printk("Failed to get reg-gps-vcn LDO, %d\n", ret);
		return ret;
	}
	pr_debug("gps get vcn28 supply ok.\n");

	ret = regulator_set_voltage(gps_vcn, 2800000, 2800000);
	if (ret != 0) {
		printk("gps: gps failed to set gps_vcn voltage: %d\n", ret);
		return ret;
	}
	pr_debug("gps set vcn28 supply ok.\n");

	ret = regulator_enable(gps_vcn);
	if (ret != 0) {
		printk("gps: Failed to enable gps_vcn: %d\n", ret);
		return ret;
	}
	/* get current voltage settings */
	volt = regulator_get_voltage(gps_vcn);
	if (volt == 2800000)
		pr_err("gps: check regulator voltage=2800000 pass!\n");
	else
		pr_err("gps: check regulator voltage=2800000 fail! (voltage: %d)\n", volt);

	return ret;
}

static int of_get_ocxo_platform_data(struct device *dev)
{
        int ret;

        if (dev->of_node) {
                const struct of_device_id *match;

                match = of_match_device(ocxo_pwr_of_match, dev);
                if (!match) {
                        printk("Error: No device match found  [ocxo_pwr_of_match]\n");
                        return -ENODEV;
                }
        }
				ret = gps_get_vcn_supply(dev);
				if(ret != 0) {
					printk("gps_get_vcn_supply get error !! %d\n", ret);	
				}

        return ret;
}
#endif


static int ocxo_pwr_probe(struct platform_device *dev)
{
#ifdef CONFIG_OF       
       if ( 0 != of_get_ocxo_platform_data(&dev->dev) ) {
               return -1;
       }
#endif
       return 0;
}


static struct platform_driver ocxo_pwr_driver = {
       .probe = ocxo_pwr_probe,
       .driver = {
                  .name = "ocxo_pwr",
                  .owner = THIS_MODULE,
#ifdef CONFIG_OF
                  .of_match_table = ocxo_pwr_of_match,
#endif
       },
};

static int __init ocxo_pwr_init(void)
{
       int ret;

       ret = platform_driver_register(&ocxo_pwr_driver);
       if (ret) {
               pr_err("%s: Unable to register driver (%d)\n", __func__, ret);
               return ret;
       }

       return 0;
}

static void __exit ocxo_pwr_exit(void)
{
}
module_init(ocxo_pwr_init);
module_exit(ocxo_pwr_exit);

MODULE_AUTHOR("Alvin");
MODULE_DESCRIPTION("Emdoor gps ocxo pwr Driver");
MODULE_LICENSE("GPL");