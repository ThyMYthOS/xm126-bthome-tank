# TODO

- Replace the default MCUboot signing key (NCS sysbuild's bundled
  `root-rsa-2048.pem` test key, used because `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE`
  is unset in `app/sysbuild.conf`) with a project-owned key before wider
  deployment -- the default key is public/checked into upstream mcuboot, so it
  provides no real protection against a forged `.signed.bin` being accepted by
  the bootloader.
