/*
 *  scope_eqtrack.h
 *  PHD Guiding
 *
 *  Created by J-E Lamiaud
 *  Copyright (c) 2019 J-E Lamiaud
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
 *    Neither the name of Bret McKee, Dad Dog Development,
 *     Craig Stark, Stark Labs nor the names of its
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

#ifdef GUIDE_EQTRACK

/*
 * EQTrack support (http://eqtrack.sourceforge.net/)
 */
class ScopeEQTrack : public Scope
{
public:
	bool m_exitSendLoop;

    ScopeEQTrack(void);
    virtual ~ScopeEQTrack(void);

    virtual bool Connect(void) override;
    virtual bool Disconnect(void) override;

    virtual bool HasNonGuiMove() override;
    virtual MOVE_RESULT Guide(GUIDE_DIRECTION direction, int durationMs) override;
    virtual void WaitMoveCompletion() override;

    void doSend();

private:
	wxString m_serialPortName;
	SerialPort *m_serialPort;
	int m_moveDuration[2];
	u_short m_command;
	u_char m_sendBuffer[4];
	pthread_t m_sendThread;
	pthread_mutex_t m_sendMutex;

	bool HasSetupDialog() const override;
	void SetupDialog() override;

	void updateBuffer(void);
};

#endif /* GUIDE_EQTRACK */
