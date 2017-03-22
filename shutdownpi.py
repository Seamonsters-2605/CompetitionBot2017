import os

def main():
    #shuts down the pi, requires putty in PATH
    os.system("plink.exe -ssh pi@raspberrypi -pw sehome \"sudo shutdown -h now\"")

if __name__ == '__main__':
    main()

# TO SHUTDOWN PI FROM GITBASH
"""
ssh-keygen -t rsa # ENTER to every field
ssh-copy-id pi@raspberrypi

ssh pi@raspberrypi "sudo shutdown -h now"
"""