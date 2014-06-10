"""--------------------------------------------------------------------
COPYRIGHT 2013 SEGWAY Inc.

Software License Agreement:

The software supplied herewith by Segway Inc. (the "Company") for its 
RMP Robotic Platforms is intended and supplied to you, the Company's 
customer, for use solely and exclusively with Segway products. The 
software is owned by the Company and/or its supplier, and is protected 
under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Segway products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
	products, programs or data not supplied or approved in writing by the Company, 
	if such claim or proceeding would have been avoided but for such combination, 
	operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   user_event_handlers.py

 \brief  This module allows the user to define how to handle events generated
		 in rmp_interface.py.

 \Platform: Cross Platform
--------------------------------------------------------------------"""
from system_defines import *

import time,sys,os, Queue

"""
Define some general parameters for the example like various commands 
"""
RMP_CMD_NO_MOTION = [RMP_MOTION_CMD_ID,0.0,0.0]
RMP_FORCE_FEEDBACK = [RMP_CFG_CMD_ID,RMP_CMD_NONE,0.0]
RMP_SET_TRACTOR = [RMP_CFG_CMD_ID,RMP_CMD_SET_OPERATIONAL_MODE,TRACTOR_REQUEST]
RMP_SET_STANDBY = [RMP_CFG_CMD_ID,RMP_CMD_SET_OPERATIONAL_MODE,STANDBY_REQUEST]

RMP_STOP_SONG = [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_NO_SONG,MOTOR_AUDIO_PLAY_NO_SONG]
RMP_SONGS = [[RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_POWER_ON_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_POWER_OFF_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_ALARM_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_MODE_UP_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_MODE_DOWN_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_ENTER_ALARM_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_EXIT_ALARM_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_FINAL_SHUTDOWN_SONG],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_CORRECT_ISSUE],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_ISSUE_CORRECTED],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_CORRECT_ISSUE_REPEATING],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_BEGINNER_ACK],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_PLAY_EXPERT_ACK],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_ENTER_FOLLOW],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_TEST_SWEEP],
			 [RMP_CFG_CMD_ID,RMP_CMD_SET_AUDIO_COMMAND,MOTOR_AUDIO_SIMULATE_MOTOR_NOISE]]

"""
This is the class that defines how to handle events passed up by the RMP class.
These events currently include:
RMP_IS_DEAD: The main loop should recongize that the RMP thread is no longer alive
			 and should try and respawn or kill the main loop
			 
RMP_TX_RDY: The RMP class can accept a new command. Commands sent before this event
			will be queued and executed asyncronously. The user should only post new
			commands once this event has been triggered
			
RMP_RSP_DATA_RDY: A response packet is ready for the user.
"""
class RMPEventHandlers:
	def __init__(self,cmd,rsp,inflags,rmpExchange):
		
		print "Initializing event handlers"
		
		"""
		Flag to run the loop
		"""	
		self._continue = True
		self.start_time = time.time()
		self.song_playing = False
		self.idx = 0
		self.cmd_queue = cmd
		self.rsp_queue = rsp
		self.inflags = inflags
		
		"""
		Listeners for feedback
		"""
		self.feedback_listeners = []

		"""
		This is the dictionary that the outflags get passed to. Each one can be
		redefined to be passed to whatever user function you would like
		"""
		self.handle_event = dict({RMP_KILL:sys.exit,
								  RMP_INIT_FAILED:self.InitFailedExit,
								  RMP_IS_DEAD:self.Kill_loop,
								  RMP_TX_RDY:self.Send_Cmd,
								  RMP_RSP_DATA_RDY:self.Get_Rsp,
								  RMP_GOTO_STANDBY:self.GotoStandby,
								  RMP_GOTO_TRACTOR:self.GotoTractor})
	"""
	Called when the robot requests a command
	"""	
	def Send_Cmd(self):
		if self.cmd_queue.empty():
			self.cmd_queue.put(RMP_FORCE_FEEDBACK)
	
	"""
	Called when feedback from the robot is received
	"""					
	def Get_Rsp(self):
		fb_dict = self.rsp_queue.get()
		for listener in self.feedback_listeners:
			listener(fb_dict)
	
	"""
	Add a feedback listener.
	"""		
	def AddListener(self, callback):
		self.feedback_listeners.append(callback)
	
	"""
	Sets the robot to Standby mode
	"""	
	def GotoStandby(self):
		self.cmd_queue.put(RMP_SET_STANDBY)
	
	"""
	Sets the robot to Tractor mode
	"""
	def GotoTractor(self):
		self.cmd_queue.put(RMP_SET_TRACTOR)
		
	"""
	Adds a command to the queue for the robot
	"""
	def AddCommand(self,command):
		self.cmd_queue.put(command)
	
	"""
	Failure procedure
	"""
	def InitFailedExit(self):
		print "RMP initialization failed...." 
		print "exiting....."
		self.inflags.put(RMP_KILL)
		self._continue = False
	
	"""
	Termination procedure
	"""
	def Kill_loop(self):
		print "Loop terminated, killing RMP thread and exiting....."
		self.inflags.put(RMP_KILL)
		self._continue = False
