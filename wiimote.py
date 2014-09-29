

import vizact
import PyWiiUse as wiiuse

#<wii>.addWiimote(device=-1)



#class wiimote():
#	def __init__(self):
#		wiimote = wii.addWiimote()
#		vizact.onsensordown(wiimote,wii.BUTTON_A,wiimote.setRumble,True) 
#		vizact.onsensorup(wiimote,wii.BUTTON_A,wiimote.setRumble,False)


wiimotes = wiiuse.init(1, 1, handle_event, handle_ctrl_status, handle_disconnect)

wii = viz.add('wiimote.dle') 
wiimote = wii.addWiimote()
vizact.onsensordown(wiimote,wii.BUTTON_A,wiimote.setRumble,True) 
vizact.onsensorup(wiimote,wii.BUTTON_A,wiimote.setRumble,False)

viz.go()
