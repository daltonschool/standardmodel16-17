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
package org.firstinspires.ftc.robotcore.internal;

import android.graphics.Camera;
import android.util.Log;
import android.view.View;

import com.vuforia.CameraDevice;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.TrackableResult;
import com.vuforia.Vuforia;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.internal.opengl.AutoConfigGLSurfaceView;

import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;

import java.lang.reflect.Field;

/**
 *
 */
public class VuforiaLocalizerImpl_SM extends VuforiaLocalizerImpl
{
    protected   Vuforia.UpdateCallbackInterface         vuforiaCallback_SM     = new VuforiaCallback_SM();
    public      boolean                                 getFrame              = false;
    public      boolean                                 getFrame2             = false;
    public      boolean                                 gotFrame              = false;
    public      ByteBuffer                              framebuf               = null;

    public VuforiaLocalizerImpl_SM(Parameters parameters) {
        super(parameters);
        Vuforia.registerCallback(vuforiaCallback_SM);
    }

    protected class VuforiaCallback_SM implements  Vuforia.UpdateCallbackInterface
    {
        @Override public synchronized void Vuforia_onUpdate(State state)
        {
            if (getFrame2) {
                Log.i("thingy", "getFrame");
                Frame f = state.getFrame();
                Image imageRGB888 = null;

                Log.i("thingy", "# img: " + f.getNumImages());
                for (int i = 0; i < f.getNumImages(); ++i) {
                    Image image = f.getImage(i);
                    Log.i("thingy", "image format: " + image.getFormat());
                    if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                        imageRGB888 = image;
                        break;
                    }
                }

                if (imageRGB888 != null) {
                    framebuf = imageRGB888.getPixels();
                    int imageWidth = imageRGB888.getWidth();
                    int imageHeight = imageRGB888.getHeight();
                    int stride = imageRGB888.getStride();
                    Log.i("thingy", "Image width: " + imageWidth);
                    Log.i("thingy", "Image height: " + imageHeight);
                    Log.i("thingy", "Image stride: " + stride);
                } else {
                    Log.i("thingy", "framebuf = null");
                    framebuf = null;
                }
                Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, false);
                gotFrame = true;
                getFrame = false;
                getFrame2 = false;
            }
            if (getFrame && !getFrame2) {
                Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
                getFrame2 = true;
            }

            Set<VuforiaTrackableImpl> notVisible = new HashSet<VuforiaTrackableImpl>();

            synchronized (loadedTrackableSets)
            {
                for (VuforiaTrackablesImpl trackables : loadedTrackableSets)
                {
                    for (VuforiaTrackable vuforiaTrackable : trackables)
                    {
                        notVisible.add((VuforiaTrackableImpl) vuforiaTrackable);
                    }
                }
            }

            for (int i = 0; i < state.getNumTrackableResults(); i++)
            {
                TrackableResult trackableResult = state.getTrackableResult(i);
                if (isObjectTrackerResult(trackableResult))
                {
                    VuforiaTrackableImpl trackable = VuforiaTrackableImpl.getTrackable(trackableResult.getTrackable());
                    notVisible.remove(trackable);
                    trackable.noteTracked(trackableResult);
                }
            }

            for (VuforiaTrackableImpl trackable : notVisible)
            {
                trackable.noteNotTracked();
            }
        }
    }
}
