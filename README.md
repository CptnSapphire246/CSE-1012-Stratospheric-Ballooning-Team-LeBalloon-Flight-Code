Before uploading the code, ensure that the PICOLO flight computer has been properly set up. Refer to _LeBalloon_Information.md_.

Note that only two folders are needed to run the PICOLO flight code: _Libraries_ and _PICOLOv1_; _ESC_Controller_ and _TestStand_Thrust_ are not required however, they have been included to provide further information on how the propeller and the test stand function independently.

_PICOLOv1_ contains the main code used during the flight and ground testing including _PICOLOv1.ino_ which is the key player in the flight code. _Libraries_ contains all the necessary libraries that are needed to be installed for the flight code to function properly. Both folders should be placed in an 'Arduino' folder or which ever folder is being used to store all .ino files on the machine.

Branch A and B contaian two unique versions of the flight code from launch day, refer to the commit histories for more details.

Once the folders have been downloaded and the flight computer has been assembled accordingly, open up the flight code (_PICOLOv1.ino_), connect the PICOLO to the machine using a USB cable, and flash the code. A start-up message should appear on the serial monitor.
