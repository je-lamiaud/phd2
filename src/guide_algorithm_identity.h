/*
 *  guide_algorithm_identity.h
 *  PHD Guiding
 *
 *  Created by Bret McKee
 *  Copyright (c) 2012 Bret McKee
 *  All rights reserved.
 *
 *  Based upon work by Craig Stark.
 *  Copyright (c) 2006-2010 Craig Stark.
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

#ifndef GUIDE_ALGORITHM_IDENTITY_H_INCLUDED
#define GUIDE_ALGORITHM_IDENTITY_H_INCLUDED

class GuideAlgorithmIdentity : public GuideAlgorithm
{
protected:
    class GuideAlgorithmIdentityConfigDialogPane : public ConfigDialogPane
    {
        GuideAlgorithmIdentity *m_pGuideAlgorithm;

    public:
        GuideAlgorithmIdentityConfigDialogPane(wxWindow *pParent, GuideAlgorithmIdentity *pGuideAlgorithm);
        ~GuideAlgorithmIdentityConfigDialogPane();

        void LoadValues() override;
        void UnloadValues() override;
    };

public:
    GuideAlgorithmIdentity(Mount *pMount, GuideAxis axis);
    ~GuideAlgorithmIdentity();
    GUIDE_ALGORITHM Algorithm() const override;

    void reset() override;
    double result(double input) override;
    ConfigDialogPane *GetConfigDialogPane(wxWindow *pParent) override;
    wxString GetSettingsSummary() const override { return "\n"; }
    wxString GetGuideAlgorithmClassName() const override { return "Identity"; }
};

#endif /* GUIDE_ALGORITHM_IDENTITY_H_INCLUDED */
