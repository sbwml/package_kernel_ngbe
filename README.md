# kmod-ngbe

## WangXun(R) Gigabit PCI Express ethernet driver for OpenWrt

## How to build

- Enter in your openwrt dir

- Get Source & building
  
  ```shell
  git clone https://github.com/sbwml/package_kernel_ngbe package/kernel/ngbe
  make menuconfig # choose Kernel modules -> Network Devices -> kmod-ngbe
  make package/kernel/ngbe/compile V=s
  ```
  
