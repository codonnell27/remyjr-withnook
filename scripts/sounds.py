#!/usr/bin/env python
import roslib; roslib.load_manifest('sound_play')
import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Bool
from remyJr.msg import bump_data
from remyJr.msg import sonar_array
sound_client = SoundClient()

# this node allows the robot to play noises when it hits something


# all of these addresses begin in sound_play/sounds
quack = sound_client.waveSound('WAV/Quack.wav')
oops = sound_client.waveSound( 'oops.wav')
bip = sound_client.waveSound('WAV/Bip.wav')
drop= sound_client.waveSound('WAV/Droplet.wav')
monkey= sound_client.waveSound('WAV/Monkey.wav')
uhoh= sound_client.waveSound('WAV/Uh oh.wav')
boing= sound_client.waveSound('WAV/Boing.wav')
indigo= sound_client.waveSound('WAV/Indigo.wav')
moof= sound_client.waveSound('WAV/moof.wav')
click= sound_client.waveSound('WAV/Single Click.wav')
volt= sound_client.waveSound('WAV/Voltage.wav')
toy= sound_client.waveSound('WAV/ChuToy.wav')
laugh= sound_client.waveSound('WAV/Laugh.wav')
newbip= sound_client.waveSound('WAV/newbip.wav')
sosumi= sound_client.waveSound('WAV/Sosumi.wav')
whit= sound_client.waveSound('WAV/Whit.wav')
klank= sound_client.waveSound('WAV/Click-Klank.wav')
log= sound_client.waveSound('WAV/Logjam.wav')
pong= sound_client.waveSound('WAV/Pong2003.wav')
temple= sound_client.waveSound('WAV/Temple.wav')
eep = sound_client.waveSound('WAV/Wild Eep.wav')
disassemble = sound_client.waveSound('No_disassemble.wav')

def playSound():
	# sound when the robot has hit something
	if terrible_mistake:
		toy.play()

def oopsListener(data):
	# decides if the robot has hit something
	global terrible_mistake
	if data.f_bump | data.b_bump | data.r_bump | data.l_bump:
		terrible_mistake = True
	else:
		terrible_mistake = False

def listener():
	rospy.Subscriber("remyjr/bump_data", bump_data, oopsListener, queue_size=1)

def setup():
	global terrible_mistake
	terrible_mistake = False
	rospy.init_node('play_sound_file')
	rospy.sleep(2)
	listener()

def mainloop():
	while not rospy.is_shutdown():
		listener()
		playSound()
		rospy.sleep(1)
	disassemble.play()
	rospy.sleep(1)

if __name__=="__main__":
	try:
		setup()
		mainloop()
	except KeyboardInterrupt:
		pass
