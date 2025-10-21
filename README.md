# Hyperport-enabled akashi-temac
Modified driver to add three sysfs files to control hyperport mode. This
disallows rest of driver from accessing MDIO bus, and forces a redundant VLAN
configuration.
