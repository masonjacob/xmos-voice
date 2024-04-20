# HOW TO REPLACE MODEL IN SDK EXAMPLES

## Prerequisites
1. Generate wakeword and command set on sensory, download both tars (zips)
2. Install the buildhost following the SDK examples documentation instructions
3. Add the buildhost to path (```PATH=$PATH:/opt/xmos/bin```)
4. Use  ```nibble_swap <model>-net.bin <model>-net.bin.nibble_swap``` to create the nibble_swap files for both the command set and wake word

## ffd
1. Drop the model files into the model folder (you can make your own subfolder, by default the .cmake looks in "usa-english")
2. Create a dummy folder with the same name as the one you put the model files in and put that folder in the ```filesystem_support``` folder, and fill it with the example output audio files (this is because the example wants output tasks to exist for some reason) 
3. Modify the ffd_sensory.cmake so that the ```set(SENSORY_COMMAND_SEARCH_HEADER_FILE...``` and other file references are correct, based on the folder you just put the model in. (Make sure that the "dev" or "prod" and the version number e.g. 7.0.0 is correct with the names of your model files)
4. Modify the ```gs_command_grammarLabel[]``` name declaration in command-...-search.c to ```gs_grammarLabel[]``` (this is the name the cmake files look for for some reason)
5. Run the build commands and the ```make example_ffd_sensory``` and ```make flash_app_example_ffd_sensory```
6. After the flash error due to being in a dev container, grab the ```example_ffd_sensory.xe``` and the ```example_ffd_sensory_data_partition.bin``` and move to local machine.
7. Flash with ```xflash --quad-spi-clock 50MHz --factory example_ffd_sensory.xe --boot-partition-size 0x100000 --data example_ffd_sensory_data_partition.bin```

## low power ffd
1. Drop the model files into the model folder, DO NOT create a seperate folder for them, just drop them directly in the model folder. 
2. Modify the ffd_sensory.cmake so that the ```set(SENSORY_COMMAND_SEARCH_HEADER_FILE...``` and other file references are correct, based on the folder you just put the model in. (Make sure that the "dev" or "prod" and the version number e.g. 7.0.0 is correct with the names of your model files)
4. Run the build commands and the ```make example_low_power_ffd_sensory``` and ```make flash_app_example_low_power_ffd_sensory```
5. After the flash error due to being in a dev container, grab the ```example_low_power_ffd_sensory.xe``` and the ```example_low_power_ffd_sensory_data_partition.bin``` and move to local machine.
6. Flash with ```xflash --quad-spi-clock 50MHz --factory example_low_power_ffd_sensory.xe --boot-partition-size 0x100000 --data example_low_power_ffd_sensory_data_partition.bin```