# Prepare Raspberry PI:

* prepare 16Gb SD Card 
* download and install imager for windows or linux (under ./deployment/raspberry) 
	* select and install raspbian lite 
* copy from folder raspbery pi, files (ssh, ssh.txt) to root of boot partition of sd card 
* boot and connect to board 
* connect with ssh to board: "ssh pi@ip_address" (for example 192.168.1.100) password: raspberry 
* change user to root with "sudo su"
* download rotem_sela_top repository as zip file rotem_sela_top-[revision].zip 
	* unpack and go to deployment folder 
	* run install.sh
	### `bash install.sh`

# Update:
* download rotem_sela_top repository as zip file rotem_sela_top-[revision].zip 
	* unpack and go to deployment folder 
	* run update.sh
	### `bash update.sh`
