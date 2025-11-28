# Hyperport-enabled akashi-temac
Modified driver to add three sysfs files to control hyperport mode. This
disallows rest of driver from accessing MDIO bus, and forces a redundant VLAN
configuration.

# Building and deploy
On both [klante-linux](https://github.com/DigicoUK/Klante-Linux/) and the [Q1 PS yocto layer](https://github.com/DigicoUK/di_con_babyshark_enghost_yocto/), the base akashi driver is used/unpacked directly from the Audinate tar distribution. The changes in this repo are vendored to both of these repos as a single patchfile (0001-hyperport.patch). Both build systems (q1/yocto and klante/shellscript hellscape) apply the patches to the base.

Unfortunately, the IP core versions are slightly different, and more importantly the kernel versions are different between the two targets. This means we have two different patches and patch bases. All changes should be cherry-picked to both parallel branches, unless they are specific to the Q1 or Klante.

<img width="1100" height="947" alt="akashi_patch" src="https://github.com/user-attachments/assets/fb4537e5-7ab7-42d6-8e7f-0e12a8d2a2ff" />

Here is how I make the patches:

Q1:
```
git diff patch-base-q1..HEAD > ~/babyshark-enghost/di_con_babyshark_enghost_yocto/meta-dante/recipes-modules/akashi-temac/files/0001-hyperport-q1.patch
```

Klante:
```
git diff patch-base HEAD > ../Klante-Linux/products/DIGICO/hyperport_akashi_patch/0001-hyperport.patch
```
