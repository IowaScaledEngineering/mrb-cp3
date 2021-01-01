

void setTurnoutPosition(uint8_t turnoutNum, uint8_t pos)
{
	if (pos)
		PORTC |= _BV(PC7);
	else
		PORTC &= ~_BV(PC7);
}

void timelockLEDOn(uint8_t turnoutNum)
{
	PORTD |= _BV(PD2);
}

void timelockLEDOff(uint8_t turnoutNum)
{
	PORTD &= ~_BV(PD2);
}

void trackShuntOn(uint8_t turnoutNum)
{
	PORTD |= _BV(PD3);
}

void trackShuntOff(uint8_t turnoutNum)
{
	PORTD &= ~_BV(PD3);
}


			// Timelocked Manual Switch Simulator
			switch(state)
			{
				// STATE_LOCKED - holds turnout in default position	
				case STATE_LOCKED:
					timelock = 0;
					setTurnoutPosition(defaultTurnoutPosition);
					trackShuntOff();
					timelockLEDOff();

					if (unlockSwitchOn(ioState))
					{
						timelock = getTimeIntervalInSecs();
						state = STATE_TIMERUN;
					}
					break;
			
				case STATE_TIMERUN:
					setTurnoutPosition(defaultTurnoutPosition);
					trackShuntOn();
					if (timerPhase) 
						timelockLEDOn();
					else
						timelockLEDOff();

					if (0 == timelock)
					{
						if (unlockSwitchOn(ioState))
						{
							state = STATE_UNLOCKED;
						} else {
							state = STATE_LOCKED;
						}
					}

					break;
			
				case STATE_UNLOCKED:
					trackShuntOn();
					timelockLEDOn();
					setTurnoutPosition(getInputTurnoutPosition());
					
					// If the user has moved the turnout back to the default position
					// and released the lock, return to the locked up state
					if (!unlockSwitchOn(ioState) 
						&& getInputTurnoutPosition() == defaultTurnoutPosition)
					{
						// Give the switch machine two seconds to lock back up
						setTurnoutPosition(defaultTurnoutPosition);
						timelock = 2;
						state = STATE_RELOCKING;
					}
					break;

				case STATE_RELOCKING:
					if (0 == timelock)
					{
						if(!unlockSwitchOn(ioState))
							state = STATE_LOCKED;
						else
							state = STATE_UNLOCKED;
					}
					break;

				default:
					state = STATE_LOCKED;
					break;
			}
