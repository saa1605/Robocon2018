Chindi errors for debugging

Coding:
1) agar Interrupt define kiya hai toh vo connected hona chahiye
2) pinsdef aur connections check karo
3) sare used pwm_inits kiye hai
4) incase of checking joystick status in any loop dont forget to put psb_read_gamepad(); to update the parameters   
5) for debouncing psb buttons dont just put empty while(); function put psb_read_gamepad(); to update the the button flags

Electronixs:
1) ground common hone chahiye
2) agar Interrupt define kiya hai toh vo connected hona chahiye
3) in case of hercules if direction lights aa raha hai toh output wires check karo else pwm check
4) code wale common pins rakhenge jo use nhi ho raha hai vo check kar
5) agar hub pe sab direction aur pwm dono voltages barabar ho toh frc check karo
6) 