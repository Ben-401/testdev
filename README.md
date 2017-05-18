# testdev

generating bitstream from compile-script:
- creates ./build-logs dir and writes upto 7x log-files each iteration,
- creates ./sdcard-files dir and copies the bit-file to this location,
- build artefacts are placed into:
-- ./isepn147/mega65/working
-- ./isepn147/mega65/working/megascript

generating bitstream from ISE:
- "mega65" is the project to load
- build artefacts are placed into:
-- ./isepn147/mega65/iseconfig
-- ./isepn147/mega65/working
