/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.external;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.VuforiaLocalizerImpl_SM;

/**
 * {@link ClassFactory_SM} provides a means by which various objects in the SDK may be logically
 * instantiated without exposing their external class identities to user's programs.
 */
public class ClassFactory_SM
    {
    /**
     * {@link #createVuforiaLocalizer_SM(VuforiaLocalizer.Parameters) createVuforiaLocalizer} instantiates
     * an instance of the Vuforia robot localization engine.
     *
     * @param parameters the parameters used to configure the new instance of the engine
     * @return a new instance of the Vuforia robot localization engine.
     *
     * @see VuforiaLocalizer
     * @see org.firstinspires.ftc.robotcore.external.navigation.Orientation
     * @see <a href="http://www.vuforia.com/">vuforia.com</a>
     */
    public static VuforiaLocalizer createVuforiaLocalizer_SM(VuforiaLocalizer.Parameters parameters)
        {
        return new VuforiaLocalizerImpl_SM(parameters);
        }
    }
