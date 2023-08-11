/*
 *  scope_eqtrack.cpp
 *  PHD Guiding
 *
 *  Created by J-E Lamiaud.
 *  Copyright (c) 2019 J-E Lamiaud.
 *  All rights reserved.
 *
 *  This source code is distributed under the following "BSD" license
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *    Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *    Neither the name of Craig Stark, Stark Labs nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <sys/time.h>

#include <wx/utils.h>

#include "phd.h"

#ifdef GUIDE_EQTRACK

#define K_North_Button      (1<<0)
#define K_South_Button      (1<<1)
#define K_West_Button       (1<<2)
#define K_East_Button       (1<<3)
#define K_Speed_Switch      (1<<4)

#define K_Tracking_Mask     (3<<5)
#define K_Tracking_Sidereal (0<<5)
#define K_Tracking_Solar    (1<<5)
#define K_Tracking_Lunar    (2<<5)
#define K_Tracking_User     (3<<5)

#define K_Tuning_Mask       (3<<8)
#define K_Tuning_Tracking   (3<<8)
#define K_Tuning_Fast_AD    (2<<8)
#define K_Tuning_Slow_D     (1<<8)
#define K_Tuning_Fast_D     (0<<8)
#define K_Tuning_Switch     (1<<10)

#define RA_MOVE 0
#define DEC_MOVE 1

#define LOOP_TIME_INCREMENT 100000000 // ns
#define MAX_TV_NSEC 1000000000

static void *sendLoop(void *arg)
{
	static struct timespec lastLoop = {0,0};
	static double sum = 0, sum2 = 0, minPeriod = HUGE_VAL, maxPeriod = -HUGE_VAL;
	static int n = -1;
	ScopeEQTrack *eq = (ScopeEQTrack*)arg;
	struct timespec deadline;

	if (clock_gettime(CLOCK_MONOTONIC, &deadline) != 0)
	{
		Debug.Write(wxString::Format("EQTrack loop: Could not get initial time\n%s\n",
			                          strerror(errno)));
		return(NULL);
	}

	while (!eq->m_exitSendLoop)
	{
		struct timespec now, duration;

		if (clock_gettime(CLOCK_MONOTONIC, &now) == 0)
		{
			if (n >= 0)
			{
				if (lastLoop.tv_nsec <= now.tv_nsec)
				{
					duration.tv_sec =  now.tv_sec - lastLoop.tv_sec;
					duration.tv_nsec = now.tv_nsec - lastLoop.tv_nsec;
				}
				else
				{
					duration.tv_sec = now.tv_sec - lastLoop.tv_sec - 1;
					duration.tv_nsec = now.tv_nsec + MAX_TV_NSEC - lastLoop.tv_nsec;
				}
				double d = (double)duration.tv_sec + (double)duration.tv_nsec/(double)MAX_TV_NSEC;
				d *= 1000.0; // Millisec
				if ( d < minPeriod)
					minPeriod = d;
				if (d > maxPeriod)
					maxPeriod = d;
				sum += d;
				sum2 += d*d;
			}
			lastLoop = now;
			n += 1;
			if (n >= 600) // Statistics every minute
			{
				double m, s;
				m = sum/(double)n;
				s = sqrt(sum2/(double)n -m*m);
         	Debug.Write(wxString::Format("EQTrack loop: mean period=%.0f sigma=%.3f min=%.0f max=%.0f\n",
         	                              m, s, minPeriod, maxPeriod));
				sum = 0.0;
				sum2 = 0.0;
				minPeriod = HUGE_VAL;
				maxPeriod = -HUGE_VAL;
				n = 0;
			}
		}

		eq->doSend();

		deadline.tv_nsec += LOOP_TIME_INCREMENT;
		if (deadline.tv_nsec >= MAX_TV_NSEC)
		{
			deadline.tv_sec += 1;
			deadline.tv_nsec -= MAX_TV_NSEC;
		}


		if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
		{
			Debug.Write(wxString::Format("EQTrack loop: Could not get current time\n%s\n",
			                             strerror(errno)));
			return(NULL);
		}
		if (deadline.tv_sec > now.tv_sec
		    || (deadline.tv_sec == now.tv_sec && deadline.tv_nsec > now.tv_nsec))
		{
			if (deadline.tv_nsec >= now.tv_nsec)
			{
				duration.tv_sec = deadline.tv_sec - now.tv_sec;
				duration.tv_nsec = deadline.tv_nsec - now.tv_nsec;
			}
			else
			{
				duration.tv_sec = deadline.tv_sec - now.tv_sec - 1;
				duration.tv_nsec = deadline.tv_nsec + MAX_TV_NSEC - now.tv_nsec;
			}
			nanosleep(&duration, NULL);
		}
		else
		{
			Debug.Write(wxString::Format("EQTrack loop: Late loop %ld.%03ld for deadline %ld.%03ld\n",
			now.tv_sec, (now.tv_nsec + 500000)/1000000,
			deadline.tv_sec, (deadline.tv_nsec + 500000)/1000000));
		}
	}

	sum = 0;
	sum2 = 0;
	n = -1;

	return(NULL);
}

ScopeEQTrack::ScopeEQTrack(void)
{
   m_Name = wxString("EQ Track by Joseph");
	m_exitSendLoop = false;
   m_serialPortName = pConfig->Profile.GetString("/scope_eqtrack/serialport", wxEmptyString);
   if (pthread_mutex_init(&m_sendMutex, NULL) != 0 )
   {
   	throw ERROR_INFO("Could not create the send loop mutex");
	}
	m_serialPort = SerialPort::SerialPortFactory();
	m_moveDuration[RA_MOVE] = 0;
	m_moveDuration[DEC_MOVE] = 0;
   m_command = 0xFFFF;
   updateBuffer();
}

ScopeEQTrack::~ScopeEQTrack(void)
{
	m_exitSendLoop = true;
   delete m_serialPort;
   pthread_mutex_destroy(&m_sendMutex);
}

bool ScopeEQTrack::HasSetupDialog() const
{
    return true;
}

void ScopeEQTrack::SetupDialog()
{
    try
    {
        wxArrayString serialPorts;
        if (m_serialPort)
        {
            serialPorts = m_serialPort->GetSerialPortList();
        }

        if (serialPorts.IsEmpty())
        {
            wxMessageBox(_("No serial ports found"),_("Error"), wxOK | wxICON_ERROR);
            throw ERROR_INFO("No Serial ports found");
        }

        int resp = serialPorts.Index(m_serialPortName);

        resp = wxGetSingleChoiceIndex(_("Select serial port"),_("Serial Port"), serialPorts,
                                      NULL, wxDefaultCoord, wxDefaultCoord, true, wxCHOICE_WIDTH, wxCHOICE_HEIGHT,
                                      resp);

        m_serialPortName = serialPorts[resp];
    }
    catch (const wxString& Msg)
    {
        POSSIBLY_UNUSED(Msg);
        m_serialPortName = wxEmptyString;
    }
}

bool ScopeEQTrack::HasNonGuiMove()
{
	return true;
}

void ScopeEQTrack::updateBuffer(void)
{
   static const u_char hamming[] =
      {0x00, 0xE1, 0xD2, 0x33, 0xB4, 0x55, 0x66, 0x87,
       0x78, 0x99, 0xAA, 0x4B, 0xCC, 0x2D, 0x1E, 0xFF};

//	printf("Buffer is : ");
   for (int i = 0; i < 4; i++ )
   {
      m_sendBuffer[i] = hamming[(m_command >> i*4) & 0x0F];
//      printf("%02X ", (m_sendBuffer[i]) & 0xFF);
   }
//   printf("\n");
}

void ScopeEQTrack::doSend(void)
{
  	m_serialPort->Send( m_sendBuffer, 4 );

	if (pthread_mutex_lock(&m_sendMutex) != 0)
		Debug.Write(wxString::Format("EQTrack::doSend: failed to acquire mutex\n%s\n",
			                          strerror(errno)));

	bool update = false;
	if (m_moveDuration[RA_MOVE] > 0)
	{
	   m_moveDuration[RA_MOVE] -= 1;
	   if (m_moveDuration[RA_MOVE] == 0)
	      m_command |= (K_West_Button | K_East_Button);
	   update = true;
	}
	if (m_moveDuration[DEC_MOVE] > 0)
	{
	   m_moveDuration[DEC_MOVE] -= 1;
	   if (m_moveDuration[DEC_MOVE] == 0)
	      m_command |= (K_North_Button | K_South_Button);
	   update = true;
	}

	if (update)
		updateBuffer();

	if (pthread_mutex_unlock(&m_sendMutex) != 0)
		Debug.Write(wxString::Format("EQTrack::doSend: failed to release mutex\n%s\n",
			                          strerror(errno)));
}

bool ScopeEQTrack::Connect()
{
   // Open the serial line
   bool bError = false;

    try
    {
      if (!m_serialPort)
      {
         throw ERROR_INFO("ScopeEQTrack::Connect: no serial port");
      }

      if (m_serialPortName.IsEmpty())
      {
         SetupDialog();
      }

      Debug.Write(wxString::Format("Connecting to EQTrack on port %s\n", m_serialPortName));

      if (m_serialPort->Connect(m_serialPortName, 1200, 8, 1, SerialPort::ParityNone, false, false))
      {
         throw ERROR_INFO("ScopeEQTrack::Connect: serial port connect failed");
      }

      wxYield();

		// Start a loop which send continuously the command
		m_exitSendLoop = false;
		pthread_attr_t loopAttrs;
		struct sched_param sched;
		pthread_attr_init(&loopAttrs);
		pthread_attr_setschedpolicy(&loopAttrs, SCHED_FIFO);
		sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
		pthread_attr_setschedparam(&loopAttrs, &sched);
		if (pthread_create(&m_sendThread, &loopAttrs, sendLoop, this) != 0)
      {
         throw ERROR_INFO("ScopeEQTrack::Connect: send loop could not be started");
      }

      pConfig->Profile.SetString("/scope_eqtrack/serialport", m_serialPortName);
   }
   catch (const wxString& Msg)
   {
        POSSIBLY_UNUSED(Msg);
        bError = true;
   }

	if (!bError)
	   bError = Scope::Connect();

   return bError;
}

bool ScopeEQTrack::Disconnect(void)
{
	Debug.Write(wxString::Format("EQ Track: disconnecting\n"));
	m_exitSendLoop = true;
	if (pthread_join(m_sendThread, NULL) != 0)
		Debug.Write(wxString::Format("EQTrack: failed to join sending thread\n"));
	m_serialPort->Disconnect();
	return Scope::Disconnect();
}

Mount::MOVE_RESULT ScopeEQTrack::Guide(GUIDE_DIRECTION direction, int duration)
{
	if (duration == 0)
		return MOVE_OK;

	u_short mask, value;
	int milliDuration = (duration < 50 ? 1 : (duration + 50) / 100); // At least one pulse
	int axis;

   switch (direction)
   {
      case EAST:
         mask = ~(K_West_Button | K_East_Button);
         value = ~K_East_Button;
         axis = RA_MOVE;
         break;
      case WEST:
         mask = ~(K_West_Button | K_East_Button);
         value = ~K_West_Button;
         axis = RA_MOVE;
         break;
      case NORTH:
         mask = ~(K_North_Button | K_South_Button);
         value = ~K_North_Button;
         axis = DEC_MOVE;
         break;
      case SOUTH:
         mask = ~(K_North_Button | K_South_Button);
         value = ~K_South_Button;
         axis = DEC_MOVE;
         break;
      case NONE:
      	// Stop all
         mask = ~(K_North_Button | K_South_Button| K_West_Button | K_East_Button);
         value = 0xFFFF;
         axis = -1;
         break;
   }

	// Allow fast manual move
	mask &= ~K_Speed_Switch;
	switch (Guider::GetExposedState())
	{
		case EXPOSED_STATE_NONE:
		case EXPOSED_STATE_SELECTED:
		case EXPOSED_STATE_PAUSED:
		case EXPOSED_STATE_LOOPING:
			if (wxGetKeyState(WXK_ALT))
				value &= ~K_Speed_Switch; // Fast move if option pressed
			break;
		default:
			value |= K_Speed_Switch; // Always a slow move
			break;
	}

	if (pthread_mutex_lock(&m_sendMutex) != 0)
		Debug.Write(wxString::Format("EQTrack::Guide: failed to acquire mutex\n%s\n",
			                          strerror(errno)));

	m_command = (m_command & mask) | (value & ~mask);

	updateBuffer();

	if (axis < 0)
	{
      m_moveDuration[RA_MOVE] = 0;
      m_moveDuration[DEC_MOVE] = 0;
   }
   else
   {
   	m_moveDuration[axis] = milliDuration;
   }

	if (pthread_mutex_unlock(&m_sendMutex) != 0)
		Debug.Write(wxString::Format("EQTrack::Guide: failed to release mutex\n%s\n",
			                          strerror(errno)));

   return MOVE_OK;
}

void ScopeEQTrack::WaitMoveCompletion()
{
	int raDuration, decDuration, duration;
	if (pthread_mutex_lock(&m_sendMutex) != 0)
		Debug.Write(wxString::Format("EQTrack::WaitMoveCompletion: failed to acquire mutex\n%s\n",
			                          strerror(errno)));

	raDuration = m_moveDuration[RA_MOVE];
	decDuration = m_moveDuration[DEC_MOVE];

	if (pthread_mutex_unlock(&m_sendMutex) != 0)
		Debug.Write(wxString::Format("EQTrack::WaitMoveCompletion: failed to release mutex\n%s\n",
			                          strerror(errno)));

	duration = (raDuration > decDuration ? raDuration : decDuration)*100;
	Debug.Write(wxString::Format("EQTrack::WaitMoveCompletion: waiting %d ms\n", duration));
   WorkerThread::MilliSleep(duration, WorkerThread::INT_ANY);
}

#endif /* GUIDE_EQTRACK */
