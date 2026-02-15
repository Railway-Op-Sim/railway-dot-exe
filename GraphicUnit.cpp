// GraphicUnit.cpp
/*
      BEWARE OF COMMENTS in .cpp files:  they were accurate when written but have
      sometimes been overtaken by changes and not updated
      Comments in .h files are believed to be accurate and up to date

      This is a source code file for "railway.exe", a railway operation
      simulator, written originally in Borland C++ Builder 4 Professional with
      later updates in Embarcadero C++Builder.
      Copyright (C) 2010 Albert Ball [original development]

      This program is free software: you can redistribute it and/or modify
      it under the terms of the GNU General Public License as published by
      the Free Software Foundation, either version 3 of the License, or
      (at your option) any later version.

      This program is distributed in the hope that it will be useful,
      but WITHOUT ANY WARRANTY; without even the implied warranty of
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
      along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
// ---------------------------------------------------------------------------

#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <Buttons.hpp>
#include <ExtCtrls.hpp>
#include <Menus.hpp>
#include <Dialogs.hpp>
#include <Graphics.hpp>
#include <ComCtrls.hpp>
#include <fstream>
#include <vector>
#include <vcl.h>

#pragma hdrstop

#include "GraphicUnit.h"
#include "Utilities.h"
#include "Modding.h"

// ---------------------------------------------------------------------------
#pragma package(smart_init)
// ---------------------------------------------------------------------------

TRailGraphics *RailGraphics;

// ---------------------------------------------------------------------------

void TRailGraphics::loadSpeedButGlyphs() {

// These are the new glyphs for v2.3.0 that stay black, they are transparent, using the bottom LH corner pixel as the transparent colour
    SpeedBut68NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut68NormBlackGlyph->Assign(gl68); // changed at v2.3.1 from 'LoadFromResourceName' for consistency
    SpeedBut69NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut69NormBlackGlyph->Assign(gl69);
    SpeedBut70NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut70NormBlackGlyph->Assign(gl70);
    SpeedBut71NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut71NormBlackGlyph->Assign(gl71);
    SpeedBut72NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut72NormBlackGlyph->Assign(gl72);
    SpeedBut73NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut73NormBlackGlyph->Assign(gl73);
    SpeedBut74NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut74NormBlackGlyph->Assign(gl74);
    SpeedBut75NormBlackGlyph = new Graphics::TBitmap;
    SpeedBut75NormBlackGlyph->Assign(gl75);

    SpeedBut68GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut68GrndBlackGlyph->Assign(bm68grounddblred);
    SpeedBut69GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut69GrndBlackGlyph->Assign(bm69grounddblred);
    SpeedBut70GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut70GrndBlackGlyph->Assign(bm70grounddblred);
    SpeedBut71GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut71GrndBlackGlyph->Assign(bm71grounddblred);
    SpeedBut72GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut72GrndBlackGlyph->Assign(bm72grounddblred);
    SpeedBut73GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut73GrndBlackGlyph->Assign(gl73grounddblred); // these have to use 'gl' graphics as bot LH corner = transparent
    SpeedBut74GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut74GrndBlackGlyph->Assign(gl74grounddblred); // colour, & 'bm' graphics have black at that position
    SpeedBut75GrndBlackGlyph = new Graphics::TBitmap;
    SpeedBut75GrndBlackGlyph->Assign(bm75grounddblred);
}

TRailGraphics::TRailGraphics()
{
// See Graphics.xlsx for details of all graphics

      bm10 = new Graphics::TBitmap;
    BlackCircle = new Graphics::TBitmap; //these new at v2.13.0
    bm100 = new Graphics::TBitmap;
    bm101 = new Graphics::TBitmap;
    bm106 = new Graphics::TBitmap;
    bm10Diverging = new Graphics::TBitmap;
    bm10Straight = new Graphics::TBitmap;
    bm11 = new Graphics::TBitmap;
    bm11Diverging = new Graphics::TBitmap;
    bm11Straight = new Graphics::TBitmap;
    bm12 = new Graphics::TBitmap;
    bm12Diverging = new Graphics::TBitmap;
    bm12Straight = new Graphics::TBitmap;
    bm13 = new Graphics::TBitmap;
    bm132 = new Graphics::TBitmap;
    bm132LeftFork = new Graphics::TBitmap;
    bm132RightFork = new Graphics::TBitmap;
    bm133 = new Graphics::TBitmap;
    bm133LeftFork = new Graphics::TBitmap;
    bm133RightFork = new Graphics::TBitmap;
    bm134 = new Graphics::TBitmap;
    bm134LeftFork = new Graphics::TBitmap;
    bm134RightFork = new Graphics::TBitmap;
    bm135 = new Graphics::TBitmap;
    bm135LeftFork = new Graphics::TBitmap;
    bm135RightFork = new Graphics::TBitmap;
    bm136 = new Graphics::TBitmap;
    bm136LeftFork = new Graphics::TBitmap;
    bm136RightFork = new Graphics::TBitmap;
    bm137 = new Graphics::TBitmap;
    bm137LeftFork = new Graphics::TBitmap;
    bm137RightFork = new Graphics::TBitmap;
    bm138 = new Graphics::TBitmap;
    bm138LeftFork = new Graphics::TBitmap;
    bm138RightFork = new Graphics::TBitmap;
    bm139 = new Graphics::TBitmap;
    bm139LeftFork = new Graphics::TBitmap;
    bm139RightFork = new Graphics::TBitmap;
    bm13Diverging = new Graphics::TBitmap;
    bm13Straight = new Graphics::TBitmap;
    bm14 = new Graphics::TBitmap;
    bm140 = new Graphics::TBitmap;
    bm141 = new Graphics::TBitmap;
    bm14Diverging = new Graphics::TBitmap;
    bm14Straight = new Graphics::TBitmap;
    bm16 = new Graphics::TBitmap;
    bm18 = new Graphics::TBitmap;
    bm20 = new Graphics::TBitmap;
    bm27 = new Graphics::TBitmap;
    bm28 = new Graphics::TBitmap;
    bm28Diverging = new Graphics::TBitmap;
    bm28Straight = new Graphics::TBitmap;
    bm29 = new Graphics::TBitmap;
    bm29Diverging = new Graphics::TBitmap;
    bm29Straight = new Graphics::TBitmap;
    bm30 = new Graphics::TBitmap;
    bm30Diverging = new Graphics::TBitmap;
    bm30Straight = new Graphics::TBitmap;
    bm31 = new Graphics::TBitmap;
    bm31Diverging = new Graphics::TBitmap;
    bm31Straight = new Graphics::TBitmap;
    bm32 = new Graphics::TBitmap;
    bm32Diverging = new Graphics::TBitmap;
    bm32Straight = new Graphics::TBitmap;
    bm33 = new Graphics::TBitmap;
    bm33Diverging = new Graphics::TBitmap;
    bm33Straight = new Graphics::TBitmap;
    bm34 = new Graphics::TBitmap;
    bm34Diverging = new Graphics::TBitmap;
    bm34Straight = new Graphics::TBitmap;
    bm35 = new Graphics::TBitmap;
    bm35Diverging = new Graphics::TBitmap;
    bm35Straight = new Graphics::TBitmap;
    bm36 = new Graphics::TBitmap;
    bm36Diverging = new Graphics::TBitmap;
    bm36Straight = new Graphics::TBitmap;
    bm37 = new Graphics::TBitmap;
    bm37Diverging = new Graphics::TBitmap;
    bm37Straight = new Graphics::TBitmap;
    bm38 = new Graphics::TBitmap;
    bm38Diverging = new Graphics::TBitmap;
    bm38Straight = new Graphics::TBitmap;
    bm39 = new Graphics::TBitmap;
    bm39Diverging = new Graphics::TBitmap;
    bm39Straight = new Graphics::TBitmap;
    bm40 = new Graphics::TBitmap;
    bm40Diverging = new Graphics::TBitmap;
    bm40Straight = new Graphics::TBitmap;
    bm41 = new Graphics::TBitmap;
    bm41Diverging = new Graphics::TBitmap;
    bm41Straight = new Graphics::TBitmap;
    bm42 = new Graphics::TBitmap;
    bm42Diverging = new Graphics::TBitmap;
    bm42Straight = new Graphics::TBitmap;
    bm43 = new Graphics::TBitmap;
    bm43Diverging = new Graphics::TBitmap;
    bm43Straight = new Graphics::TBitmap;
    bm45 = new Graphics::TBitmap;
    bm46 = new Graphics::TBitmap;
    bm50 = new Graphics::TBitmap;
    bm51 = new Graphics::TBitmap;
    bm53 = new Graphics::TBitmap;
    bm54 = new Graphics::TBitmap;
    bm56 = new Graphics::TBitmap;
    bm59 = new Graphics::TBitmap;
    bm65 = new Graphics::TBitmap;
    bm68CallingOn = new Graphics::TBitmap;
    bm68dblyellow = new Graphics::TBitmap;
    bm68grounddblred = new Graphics::TBitmap;
    bm68grounddblwhite = new Graphics::TBitmap;
    bm68green = new Graphics::TBitmap;
    bm68yellow = new Graphics::TBitmap;
    bm69CallingOn = new Graphics::TBitmap;
    bm69dblyellow = new Graphics::TBitmap;
    bm69grounddblred = new Graphics::TBitmap;
    bm69grounddblwhite = new Graphics::TBitmap;
    bm69green = new Graphics::TBitmap;
    bm69yellow = new Graphics::TBitmap;
    bm7 = new Graphics::TBitmap;
    bm70CallingOn = new Graphics::TBitmap;
    bm70dblyellow = new Graphics::TBitmap;
    bm70grounddblred = new Graphics::TBitmap;
    bm70grounddblwhite = new Graphics::TBitmap;
    bm70green = new Graphics::TBitmap;
    bm70yellow = new Graphics::TBitmap;
    bm71CallingOn = new Graphics::TBitmap;
    bm71dblyellow = new Graphics::TBitmap;
    bm71grounddblred = new Graphics::TBitmap;
    bm71grounddblwhite = new Graphics::TBitmap;
    bm71green = new Graphics::TBitmap;
    bm71yellow = new Graphics::TBitmap;
    bm72CallingOn = new Graphics::TBitmap;
    bm72dblyellow = new Graphics::TBitmap;
    bm72grounddblred = new Graphics::TBitmap;
    bm72grounddblwhite = new Graphics::TBitmap;
    bm72green = new Graphics::TBitmap;
    bm72yellow = new Graphics::TBitmap;
    bm73 = new Graphics::TBitmap;
    bm73CallingOn = new Graphics::TBitmap;
    bm73dblyellow = new Graphics::TBitmap;
    bm73grounddblred = new Graphics::TBitmap;
    bm73grounddblwhite = new Graphics::TBitmap;
    bm73green = new Graphics::TBitmap;
    bm73yellow = new Graphics::TBitmap;
    bm74 = new Graphics::TBitmap;
    bm74CallingOn = new Graphics::TBitmap;
    bm74dblyellow = new Graphics::TBitmap;
    bm74grounddblred = new Graphics::TBitmap;
    bm74grounddblwhite = new Graphics::TBitmap;
    bm74green = new Graphics::TBitmap;
    bm74yellow = new Graphics::TBitmap;
    bm75CallingOn = new Graphics::TBitmap;
    bm75dblyellow = new Graphics::TBitmap;
    bm75grounddblred = new Graphics::TBitmap;
    bm75grounddblwhite = new Graphics::TBitmap;
    bm75green = new Graphics::TBitmap;
    bm75yellow = new Graphics::TBitmap;
    bm77 = new Graphics::TBitmap;
    bm77Striped = new Graphics::TBitmap;
    bm78 = new Graphics::TBitmap;
    bm78Striped = new Graphics::TBitmap;
    bm7Diverging = new Graphics::TBitmap;
    bm7Straight = new Graphics::TBitmap;
    bm8 = new Graphics::TBitmap;
    bm85 = new Graphics::TBitmap;
    bm8Diverging = new Graphics::TBitmap;
    bm8Straight = new Graphics::TBitmap;
    bm9 = new Graphics::TBitmap;
    bm93set = new Graphics::TBitmap;
    bm93unset = new Graphics::TBitmap;
    bm94set = new Graphics::TBitmap;
    bm94unset = new Graphics::TBitmap;
    bm9Diverging = new Graphics::TBitmap;
    bm9Straight = new Graphics::TBitmap;
    bmGreenEllipse = new Graphics::TBitmap;
    bmGreenRect = new Graphics::TBitmap;
    bmGrid = new Graphics::TBitmap;
    bmLightBlueRect = new Graphics::TBitmap;
    bmName = new Graphics::TBitmap;
    bmNameStriped = new Graphics::TBitmap;
    bmRedEllipse = new Graphics::TBitmap;
    bmRedRect = new Graphics::TBitmap;
    bmRtCancELink1 = new Graphics::TBitmap;
    bmRtCancELink2 = new Graphics::TBitmap;
    bmRtCancELink3 = new Graphics::TBitmap;
    bmRtCancELink4 = new Graphics::TBitmap;
    bmRtCancELink6 = new Graphics::TBitmap;
    bmRtCancELink7 = new Graphics::TBitmap;
    bmRtCancELink8 = new Graphics::TBitmap;
    bmRtCancELink9 = new Graphics::TBitmap;
    br1 = new Graphics::TBitmap;
    br10 = new Graphics::TBitmap;
    br11 = new Graphics::TBitmap;
    br12 = new Graphics::TBitmap;
    br2 = new Graphics::TBitmap;
    br3 = new Graphics::TBitmap;
    br4 = new Graphics::TBitmap;
    br5 = new Graphics::TBitmap;
    br6 = new Graphics::TBitmap;
    br7 = new Graphics::TBitmap;
    br8 = new Graphics::TBitmap;
    br9 = new Graphics::TBitmap;
    Concourse = new Graphics::TBitmap;
    ConcourseGlyph = new Graphics::TBitmap;
    ConcourseStriped = new Graphics::TBitmap;
    CouplingExit1 = new Graphics::TBitmap;        //new multiplayer coupled exit graphics
    CouplingExit2 = new Graphics::TBitmap;
    CouplingExit3 = new Graphics::TBitmap;
    CouplingExit4 = new Graphics::TBitmap;
    CouplingExit6 = new Graphics::TBitmap;
    CouplingExit7 = new Graphics::TBitmap;
    CouplingExit8 = new Graphics::TBitmap;
    CouplingExit9 = new Graphics::TBitmap;
    SolidCircleRed = new Graphics::TBitmap;  //new solid circles for multiplayer
    SolidCircleYellow = new Graphics::TBitmap;
    SolidCircleGreen = new Graphics::TBitmap;
    ELk1 = new Graphics::TBitmap;
    ELk2 = new Graphics::TBitmap;
    ELk3 = new Graphics::TBitmap;
    ELk4 = new Graphics::TBitmap;
    ELk6 = new Graphics::TBitmap;
    ELk7 = new Graphics::TBitmap;
    ELk8 = new Graphics::TBitmap;
    ELk9 = new Graphics::TBitmap;
    BlackOctagon = new Graphics::TBitmap; //these new at v2.13.0
    gl1 = new Graphics::TBitmap;
    gl10 = new Graphics::TBitmap;
    gl100 = new Graphics::TBitmap;
    gl101 = new Graphics::TBitmap;
    gl102 = new Graphics::TBitmap;
    gl103 = new Graphics::TBitmap;
    gl104 = new Graphics::TBitmap;
    gl105 = new Graphics::TBitmap;
    gl106 = new Graphics::TBitmap;
    gl107 = new Graphics::TBitmap;
    gl108 = new Graphics::TBitmap;
    gl109 = new Graphics::TBitmap;
    gl11 = new Graphics::TBitmap;
    gl110 = new Graphics::TBitmap;
    gl111 = new Graphics::TBitmap;
    gl112 = new Graphics::TBitmap;
    gl113 = new Graphics::TBitmap;
    gl114 = new Graphics::TBitmap;
    gl115 = new Graphics::TBitmap;
    gl116 = new Graphics::TBitmap;
    gl117 = new Graphics::TBitmap;
    gl118 = new Graphics::TBitmap;
    gl119 = new Graphics::TBitmap;
    gl12 = new Graphics::TBitmap;
    gl120 = new Graphics::TBitmap;
    gl121 = new Graphics::TBitmap;
    gl122 = new Graphics::TBitmap;
    gl123 = new Graphics::TBitmap;
    gl124 = new Graphics::TBitmap;
    gl125 = new Graphics::TBitmap;
    gl126 = new Graphics::TBitmap;
    gl127 = new Graphics::TBitmap;
    gl128 = new Graphics::TBitmap;
    gl129 = new Graphics::TBitmap;
    gl129Striped = new Graphics::TBitmap;
    gl13 = new Graphics::TBitmap;
    gl130 = new Graphics::TBitmap;
    gl130Striped = new Graphics::TBitmap;
    gl131 = new Graphics::TBitmap;
    gl132 = new Graphics::TBitmap;
    gl133 = new Graphics::TBitmap;
    gl134 = new Graphics::TBitmap;
    gl135 = new Graphics::TBitmap;
    gl136 = new Graphics::TBitmap;
    gl137 = new Graphics::TBitmap;
    gl138 = new Graphics::TBitmap;
    gl139 = new Graphics::TBitmap;
    gl14 = new Graphics::TBitmap;
    gl140 = new Graphics::TBitmap;
    gl141 = new Graphics::TBitmap;
    gl142 = new Graphics::TBitmap;
    gl143 = new Graphics::TBitmap;
    gl145 = new Graphics::TBitmap;
    gl145Striped = new Graphics::TBitmap;
    gl146 = new Graphics::TBitmap;
    gl146Striped = new Graphics::TBitmap;
    gl15 = new Graphics::TBitmap;
    gl16 = new Graphics::TBitmap;
    gl18 = new Graphics::TBitmap;
    gl19 = new Graphics::TBitmap;
    gl2 = new Graphics::TBitmap;
    gl20 = new Graphics::TBitmap;
    gl21 = new Graphics::TBitmap;
    gl22 = new Graphics::TBitmap;
    gl23 = new Graphics::TBitmap;
    gl24 = new Graphics::TBitmap;
    gl25 = new Graphics::TBitmap;
    gl26 = new Graphics::TBitmap;
    gl27 = new Graphics::TBitmap;
    gl28 = new Graphics::TBitmap;
    gl29 = new Graphics::TBitmap;
    gl3 = new Graphics::TBitmap;
    gl30 = new Graphics::TBitmap;
    gl31 = new Graphics::TBitmap;
    gl32 = new Graphics::TBitmap;
    gl33 = new Graphics::TBitmap;
    gl34 = new Graphics::TBitmap;
    gl35 = new Graphics::TBitmap;
    gl36 = new Graphics::TBitmap;
    gl37 = new Graphics::TBitmap;
    gl38 = new Graphics::TBitmap;
    gl39 = new Graphics::TBitmap;
    gl4 = new Graphics::TBitmap;
    gl40 = new Graphics::TBitmap;
    gl41 = new Graphics::TBitmap;
    gl42 = new Graphics::TBitmap;
    gl43 = new Graphics::TBitmap;
    gl44 = new Graphics::TBitmap;
    gl45 = new Graphics::TBitmap;
    gl46 = new Graphics::TBitmap;
    gl47 = new Graphics::TBitmap;
    gl48 = new Graphics::TBitmap;
    gl49 = new Graphics::TBitmap;
    gl5 = new Graphics::TBitmap;
    gl50 = new Graphics::TBitmap;
    gl51 = new Graphics::TBitmap;
    gl52 = new Graphics::TBitmap;
    gl53 = new Graphics::TBitmap;
    gl54 = new Graphics::TBitmap;
    gl55 = new Graphics::TBitmap;
    gl56 = new Graphics::TBitmap;
    gl57 = new Graphics::TBitmap;
    gl58 = new Graphics::TBitmap;
    gl59 = new Graphics::TBitmap;
    gl6 = new Graphics::TBitmap;
    gl60 = new Graphics::TBitmap;
    gl61 = new Graphics::TBitmap;
    gl62 = new Graphics::TBitmap;
    gl63 = new Graphics::TBitmap;
    gl64 = new Graphics::TBitmap;
    gl65 = new Graphics::TBitmap;
    gl66 = new Graphics::TBitmap;
    gl67 = new Graphics::TBitmap;
    gl68 = new Graphics::TBitmap;
    gl69 = new Graphics::TBitmap;
    gl7 = new Graphics::TBitmap;
    gl70 = new Graphics::TBitmap;
    gl71 = new Graphics::TBitmap;
    gl72 = new Graphics::TBitmap;
    gl73 = new Graphics::TBitmap;
    gl73grounddblred = new Graphics::TBitmap;
    gl74 = new Graphics::TBitmap;
    gl74grounddblred = new Graphics::TBitmap;
    gl75 = new Graphics::TBitmap;
    gl76 = new Graphics::TBitmap;
    gl76Striped = new Graphics::TBitmap;
    gl77 = new Graphics::TBitmap;
    gl78 = new Graphics::TBitmap;
    gl79 = new Graphics::TBitmap;
    gl79Striped = new Graphics::TBitmap;
    gl8 = new Graphics::TBitmap;
    gl80 = new Graphics::TBitmap;
    gl81 = new Graphics::TBitmap;
    gl82 = new Graphics::TBitmap;
    gl83 = new Graphics::TBitmap;
    gl84 = new Graphics::TBitmap;
    gl85 = new Graphics::TBitmap;
    gl86 = new Graphics::TBitmap;
    gl87 = new Graphics::TBitmap;
    gl88set = new Graphics::TBitmap;
    gl88unset = new Graphics::TBitmap;
    gl89set = new Graphics::TBitmap;
    gl89unset = new Graphics::TBitmap;
    gl9 = new Graphics::TBitmap;
    gl90set = new Graphics::TBitmap;
    gl90unset = new Graphics::TBitmap;
    gl91set = new Graphics::TBitmap;
    gl91unset = new Graphics::TBitmap;
    gl92set = new Graphics::TBitmap;
    gl92unset = new Graphics::TBitmap;
    gl93set = new Graphics::TBitmap;
    gl94set = new Graphics::TBitmap;
    gl95set = new Graphics::TBitmap;
    gl95unset = new Graphics::TBitmap;
    gl97 = new Graphics::TBitmap;
    gl98 = new Graphics::TBitmap;
    gl99 = new Graphics::TBitmap;
    Plat68 = new Graphics::TBitmap;
    Plat68Striped = new Graphics::TBitmap;
    Plat69 = new Graphics::TBitmap;
    Plat69Striped = new Graphics::TBitmap;
    Plat70 = new Graphics::TBitmap;
    Plat70Striped = new Graphics::TBitmap;
    Plat71 = new Graphics::TBitmap;
    Plat71Striped = new Graphics::TBitmap;
    sm1 = new Graphics::TBitmap;
    sm10 = new Graphics::TBitmap;
    sm100 = new Graphics::TBitmap;
    sm101 = new Graphics::TBitmap;
    sm102 = new Graphics::TBitmap;
    sm103 = new Graphics::TBitmap;
    sm104 = new Graphics::TBitmap;
    sm105 = new Graphics::TBitmap;
    sm106 = new Graphics::TBitmap;
    sm107 = new Graphics::TBitmap;
    sm108 = new Graphics::TBitmap;
    sm109 = new Graphics::TBitmap;
    sm11 = new Graphics::TBitmap;
    sm110 = new Graphics::TBitmap;
    sm111 = new Graphics::TBitmap;
    sm112 = new Graphics::TBitmap;
    sm115 = new Graphics::TBitmap;
    sm117 = new Graphics::TBitmap;
    sm12 = new Graphics::TBitmap;
    sm129 = new Graphics::TBitmap;
    sm129striped = new Graphics::TBitmap;
    sm13 = new Graphics::TBitmap;
    sm130 = new Graphics::TBitmap;
    sm130striped = new Graphics::TBitmap;
    sm131striped = new Graphics::TBitmap;
    sm132 = new Graphics::TBitmap;
    sm133 = new Graphics::TBitmap;
    sm134 = new Graphics::TBitmap;
    sm135 = new Graphics::TBitmap;
    sm136 = new Graphics::TBitmap;
    sm137 = new Graphics::TBitmap;
    sm138 = new Graphics::TBitmap;
    sm139 = new Graphics::TBitmap;
    sm14 = new Graphics::TBitmap;
    sm15 = new Graphics::TBitmap;
    sm16 = new Graphics::TBitmap;
    sm18 = new Graphics::TBitmap;
    sm19 = new Graphics::TBitmap;
    sm2 = new Graphics::TBitmap;
    sm20 = new Graphics::TBitmap;
    sm21 = new Graphics::TBitmap;
    sm22 = new Graphics::TBitmap;
    sm23 = new Graphics::TBitmap;
    sm24 = new Graphics::TBitmap;
    sm25 = new Graphics::TBitmap;
    sm26 = new Graphics::TBitmap;
    sm27 = new Graphics::TBitmap;
    sm28 = new Graphics::TBitmap;
    sm29 = new Graphics::TBitmap;
    sm3 = new Graphics::TBitmap;
    sm30 = new Graphics::TBitmap;
    sm31 = new Graphics::TBitmap;
    sm32 = new Graphics::TBitmap;
    sm33 = new Graphics::TBitmap;
    sm34 = new Graphics::TBitmap;
    sm35 = new Graphics::TBitmap;
    sm36 = new Graphics::TBitmap;
    sm37 = new Graphics::TBitmap;
    sm38 = new Graphics::TBitmap;
    sm39 = new Graphics::TBitmap;
    sm4 = new Graphics::TBitmap;
    sm40 = new Graphics::TBitmap;
    sm41 = new Graphics::TBitmap;
    sm42 = new Graphics::TBitmap;
    sm43 = new Graphics::TBitmap;
    sm44 = new Graphics::TBitmap;
    sm45 = new Graphics::TBitmap;
    sm46 = new Graphics::TBitmap;
    sm47 = new Graphics::TBitmap;
    sm48 = new Graphics::TBitmap;
    sm49 = new Graphics::TBitmap;
    sm5 = new Graphics::TBitmap;
    sm50 = new Graphics::TBitmap;
    sm51 = new Graphics::TBitmap;
    sm52 = new Graphics::TBitmap;
    sm53 = new Graphics::TBitmap;
    sm54 = new Graphics::TBitmap;
    sm55 = new Graphics::TBitmap;
    sm56 = new Graphics::TBitmap;
    sm57 = new Graphics::TBitmap;
    sm58 = new Graphics::TBitmap;
    sm59 = new Graphics::TBitmap;
    sm6 = new Graphics::TBitmap;
    sm60 = new Graphics::TBitmap;
    sm61 = new Graphics::TBitmap;
    sm62 = new Graphics::TBitmap;
    sm63 = new Graphics::TBitmap;
    sm64 = new Graphics::TBitmap;
    sm65 = new Graphics::TBitmap;
    sm66 = new Graphics::TBitmap;
    sm67 = new Graphics::TBitmap;
    sm7 = new Graphics::TBitmap;
    sm76 = new Graphics::TBitmap;
    sm76striped = new Graphics::TBitmap;
    sm77 = new Graphics::TBitmap;
    sm77striped = new Graphics::TBitmap;
    sm78 = new Graphics::TBitmap;
    sm78striped = new Graphics::TBitmap;
    sm79 = new Graphics::TBitmap;
    sm79striped = new Graphics::TBitmap;
    sm8 = new Graphics::TBitmap;
    sm80 = new Graphics::TBitmap;
    sm81 = new Graphics::TBitmap;
    sm82 = new Graphics::TBitmap;
    sm83 = new Graphics::TBitmap;
    sm84 = new Graphics::TBitmap;
    sm85 = new Graphics::TBitmap;
    sm86 = new Graphics::TBitmap;
    sm87 = new Graphics::TBitmap;
    sm88 = new Graphics::TBitmap;
    sm89 = new Graphics::TBitmap;
    sm9 = new Graphics::TBitmap;
    sm90 = new Graphics::TBitmap;
    sm91 = new Graphics::TBitmap;
    sm92 = new Graphics::TBitmap;
    sm93 = new Graphics::TBitmap;
    sm94 = new Graphics::TBitmap;
    sm95 = new Graphics::TBitmap;
    sm96 = new Graphics::TBitmap;
    sm96striped = new Graphics::TBitmap;
    sm97 = new Graphics::TBitmap;
    sm98 = new Graphics::TBitmap;
    sm99 = new Graphics::TBitmap;
    smBlack = new Graphics::TBitmap;
    smBlue = new Graphics::TBitmap;
    smBrightGreen = new Graphics::TBitmap;
    smCaramel = new Graphics::TBitmap;
    smCyan = new Graphics::TBitmap;
    smLC = new Graphics::TBitmap;      //added v2.9.0 to show in zoom out mode
    smLightBlue = new Graphics::TBitmap;
    smMagenta = new Graphics::TBitmap;
    smName = new Graphics::TBitmap;
    smOrange = new Graphics::TBitmap;
    smPaleGreen = new Graphics::TBitmap;
    smRed = new Graphics::TBitmap;
    smYellow = new Graphics::TBitmap;
    smTransparent = new Graphics::TBitmap;
    TempBackground = new Graphics::TBitmap;
    TempHeadCode = new Graphics::TBitmap;
    UnderHFootbridge = new Graphics::TBitmap;
    UnderVFootbridge = new Graphics::TBitmap;
    FSig68 = new Graphics::TBitmap;
    FSig69 = new Graphics::TBitmap;
    FSig70 = new Graphics::TBitmap;
    FSig71 = new Graphics::TBitmap;
    FSig72 = new Graphics::TBitmap;
    FSig73 = new Graphics::TBitmap;
    FSig74 = new Graphics::TBitmap;
    FSig75 = new Graphics::TBitmap;
    FGSig68 = new Graphics::TBitmap;
    FGSig69 = new Graphics::TBitmap;
    FGSig70 = new Graphics::TBitmap;
    FGSig71 = new Graphics::TBitmap;
    FGSig72 = new Graphics::TBitmap;
    FGSig73 = new Graphics::TBitmap;
    FGSig74 = new Graphics::TBitmap;
    FGSig75 = new Graphics::TBitmap;
    bmTransparentBgnd = new Graphics::TBitmap;
    LCBothHor = new Graphics::TBitmap;
    LCBothHorMan = new Graphics::TBitmap;
    LCBotHor = new Graphics::TBitmap;
    LCBotHorMan = new Graphics::TBitmap;
    LCBothVer = new Graphics::TBitmap;
    LCBothVerMan = new Graphics::TBitmap;
    LCLHSVer = new Graphics::TBitmap;
    LCLHSVerMan = new Graphics::TBitmap;
    LCPlain = new Graphics::TBitmap;
    LCPlainMan = new Graphics::TBitmap;
    LCRHSVer = new Graphics::TBitmap;
    LCRHSVerMan = new Graphics::TBitmap;
    LCTopHor = new Graphics::TBitmap;
    LCTopHorMan = new Graphics::TBitmap;
    HeatMapGraphic = new Graphics::TBitmap; //new at v2.22.0 for length & speed heatmaps, copies existing graphic prior to colour being added
    Code_a = new Graphics::TBitmap;
    Code_b = new Graphics::TBitmap;
    Code_c = new Graphics::TBitmap;
    Code_d = new Graphics::TBitmap;
    Code_e = new Graphics::TBitmap;
    Code_f = new Graphics::TBitmap;
    Code_g = new Graphics::TBitmap;
    Code_h = new Graphics::TBitmap;
    Code_i = new Graphics::TBitmap;
    Code_j = new Graphics::TBitmap;
    Code_k = new Graphics::TBitmap;
    Code_l = new Graphics::TBitmap;
    Code_m = new Graphics::TBitmap;
    Code_n = new Graphics::TBitmap;
    Code_o = new Graphics::TBitmap;
    Code_p = new Graphics::TBitmap;
    Code_q = new Graphics::TBitmap;
    Code_r = new Graphics::TBitmap;
    Code_s = new Graphics::TBitmap;
    Code_t = new Graphics::TBitmap;
    Code_u = new Graphics::TBitmap;
    Code_v = new Graphics::TBitmap;
    Code_w = new Graphics::TBitmap;
    Code_x = new Graphics::TBitmap;
    Code_y = new Graphics::TBitmap;
    Code_z = new Graphics::TBitmap;
    Code0 = new Graphics::TBitmap;
    Code1 = new Graphics::TBitmap;
    Code2 = new Graphics::TBitmap;
    Code3 = new Graphics::TBitmap;
    Code4 = new Graphics::TBitmap;
    Code5 = new Graphics::TBitmap;
    Code6 = new Graphics::TBitmap;
    Code7 = new Graphics::TBitmap;
    Code8 = new Graphics::TBitmap;
    Code9 = new Graphics::TBitmap;
    CodeA = new Graphics::TBitmap;
    CodeB = new Graphics::TBitmap;
    CodeC = new Graphics::TBitmap;
    CodeD = new Graphics::TBitmap;
    CodeE = new Graphics::TBitmap;
    CodeF = new Graphics::TBitmap;
    CodeG = new Graphics::TBitmap;
    CodeH = new Graphics::TBitmap;
    CodeI = new Graphics::TBitmap;
    CodeJ = new Graphics::TBitmap;
    CodeK = new Graphics::TBitmap;
    CodeL = new Graphics::TBitmap;
    CodeM = new Graphics::TBitmap;
    CodeN = new Graphics::TBitmap;
    CodeO = new Graphics::TBitmap;
    CodeP = new Graphics::TBitmap;
    CodeQ = new Graphics::TBitmap;
    CodeR = new Graphics::TBitmap;
    CodeS = new Graphics::TBitmap;
    CodeT = new Graphics::TBitmap;
    CodeU = new Graphics::TBitmap;
    CodeV = new Graphics::TBitmap;
    CodeW = new Graphics::TBitmap;
    CodeX = new Graphics::TBitmap;
    CodeY = new Graphics::TBitmap;
    CodeZ = new Graphics::TBitmap;
    bmSolidBgnd = new Graphics::TBitmap;
    smSolidBgnd = new Graphics::TBitmap;
    bmDiagonalSignalBlank = new Graphics::TBitmap;
    bmPointBlank = new Graphics::TBitmap;
    bmStraightEWSignalBlank = new Graphics::TBitmap;
	bmStraightNSSignalBlank = new Graphics::TBitmap;

	loadSpeedButGlyphs();

    // GridBitmap is a 10 x 9 grid image, quicker to plot these for whole screen than small ones
    GridBitmap = new Graphics::TBitmap;
    GridBitmap->PixelFormat = pf8bit;
    GridBitmap->Width = 160;
    GridBitmap->Height = 144;
    TRect Source(0, 0, 16, 16);

    for(int x = 0; x < 10; x++)
    {
        for(int y = 0; y < 9; y++)
        {
            TRect Dest(x * 16, y * 16, (x * 16) + 16, (y * 16) + 16);
            GridBitmap->Canvas->CopyRect(Dest, bmGrid->Canvas, Source);
        }
    }
    GridBitmap->Transparent = true;
    GridBitmap->TransparentColor = clB5G5R5;

    for(int x = 0; x < 30; x++)
    {
        LinkPrefDirGraphicsPtr[x] = new Graphics::TBitmap;
        LinkPrefDirGraphicsPtr[x]->PixelFormat = pf8bit;
        LinkNonSigRouteGraphicsPtr[x] = new Graphics::TBitmap;
        LinkNonSigRouteGraphicsPtr[x]->PixelFormat = pf8bit;
        LinkSigRouteGraphicsPtr[x] = new Graphics::TBitmap;
        LinkSigRouteGraphicsPtr[x]->PixelFormat = pf8bit;
        LinkRouteAutoSigsGraphicsPtr[x] = new Graphics::TBitmap;
        LinkRouteAutoSigsGraphicsPtr[x]->PixelFormat = pf8bit;
        LinkPrefDirGraphicsPtr[x]->Width = 16;
        LinkPrefDirGraphicsPtr[x]->Height = 16;
        LinkNonSigRouteGraphicsPtr[x]->Width = 16;
        LinkNonSigRouteGraphicsPtr[x]->Height = 16;
        LinkSigRouteGraphicsPtr[x]->Width = 16;
        LinkSigRouteGraphicsPtr[x]->Height = 16;
        LinkRouteAutoSigsGraphicsPtr[x]->Width = 16;
        LinkRouteAutoSigsGraphicsPtr[x]->Height = 16;
        LinkPrefDirGraphicsPtr[x]->Transparent = true;
        LinkNonSigRouteGraphicsPtr[x]->Transparent = true;
        LinkSigRouteGraphicsPtr[x]->Transparent = true;
        LinkRouteAutoSigsGraphicsPtr[x]->Transparent = true;
    }
    for(int x = 0; x < 12; x++)
    {
        BridgePrefDirGraphicsPtr[x] = new Graphics::TBitmap;
        BridgePrefDirGraphicsPtr[x]->PixelFormat = pf8bit;
        BridgeNonSigRouteGraphicsPtr[x] = new Graphics::TBitmap;
        BridgeNonSigRouteGraphicsPtr[x]->PixelFormat = pf8bit;
        BridgeSigRouteGraphicsPtr[x] = new Graphics::TBitmap;
        BridgeSigRouteGraphicsPtr[x]->PixelFormat = pf8bit;
        BridgeRouteAutoSigsGraphicsPtr[x] = new Graphics::TBitmap;
        BridgeRouteAutoSigsGraphicsPtr[x]->PixelFormat = pf8bit;
        BridgePrefDirGraphicsPtr[x]->Width = 16;
        BridgePrefDirGraphicsPtr[x]->Height = 16;
        BridgeNonSigRouteGraphicsPtr[x]->Width = 16;
        BridgeNonSigRouteGraphicsPtr[x]->Height = 16;
        BridgeSigRouteGraphicsPtr[x]->Width = 16;
        BridgeSigRouteGraphicsPtr[x]->Height = 16;
        BridgeRouteAutoSigsGraphicsPtr[x]->Width = 16;
        BridgeRouteAutoSigsGraphicsPtr[x]->Height = 16;
        BridgePrefDirGraphicsPtr[x]->Transparent = true;
        BridgeNonSigRouteGraphicsPtr[x]->Transparent = true;
        BridgeSigRouteGraphicsPtr[x]->Transparent = true;
        BridgeRouteAutoSigsGraphicsPtr[x]->Transparent = true;
    }

    for(int x = 0; x < 10; x++)
    {
        DirectionPrefDirGraphicsPtr[x] = new Graphics::TBitmap;
        DirectionPrefDirGraphicsPtr[x]->PixelFormat = pf8bit;
        DirectionNonSigRouteGraphicsPtr[x] = new Graphics::TBitmap;
        DirectionNonSigRouteGraphicsPtr[x]->PixelFormat = pf8bit;
        DirectionSigRouteGraphicsPtr[x] = new Graphics::TBitmap;
        DirectionSigRouteGraphicsPtr[x]->PixelFormat = pf8bit;
        DirectionRouteAutoSigsGraphicsPtr[x] = new Graphics::TBitmap;
        DirectionRouteAutoSigsGraphicsPtr[x]->PixelFormat = pf8bit;
        DirectionPrefDirGraphicsPtr[x]->Width = 16;
        DirectionPrefDirGraphicsPtr[x]->Height = 16;
        DirectionNonSigRouteGraphicsPtr[x]->Width = 16;
        DirectionNonSigRouteGraphicsPtr[x]->Height = 16;
        DirectionSigRouteGraphicsPtr[x]->Width = 16;
        DirectionSigRouteGraphicsPtr[x]->Height = 16;
        DirectionRouteAutoSigsGraphicsPtr[x]->Width = 16;
        DirectionRouteAutoSigsGraphicsPtr[x]->Height = 16;
        DirectionPrefDirGraphicsPtr[x]->Transparent = true;
        DirectionNonSigRouteGraphicsPtr[x]->Transparent = true;
        DirectionSigRouteGraphicsPtr[x]->Transparent = true;
        DirectionRouteAutoSigsGraphicsPtr[x]->Transparent = true;
    }

    // points' fillets
    for(int x = 0; x < 32; x++)
    {
        PointModeGraphicsPtr[x][0] = new Graphics::TBitmap;
        PointModeGraphicsPtr[x][0]->PixelFormat = pf8bit;
        PointModeGraphicsPtr[x][1] = new Graphics::TBitmap;
        PointModeGraphicsPtr[x][1]->PixelFormat = pf8bit;
        PointModeGraphicsPtr[x][0]->Width = 16;
        PointModeGraphicsPtr[x][0]->Height = 16;
        PointModeGraphicsPtr[x][1]->Width = 16;
        PointModeGraphicsPtr[x][1]->Height = 16;
        PointModeGraphicsPtr[x][0]->Transparent = true;
        PointModeGraphicsPtr[x][1]->Transparent = true;
    }

    // Note: LockedRouteCancelPtr[0] & [5] unused
    // The following are just pointer copies, not new graphics
    LockedRouteCancelPtr[0] = bmRtCancELink1;
    LockedRouteCancelPtr[1] = bmRtCancELink1;
    LockedRouteCancelPtr[2] = bmRtCancELink2;
    LockedRouteCancelPtr[3] = bmRtCancELink3;
    LockedRouteCancelPtr[4] = bmRtCancELink4;
    LockedRouteCancelPtr[5] = bmRtCancELink4;
    LockedRouteCancelPtr[6] = bmRtCancELink6;
    LockedRouteCancelPtr[7] = bmRtCancELink7;
    LockedRouteCancelPtr[8] = bmRtCancELink8;
    LockedRouteCancelPtr[9] = bmRtCancELink9;

    // LinkGraphicsPtr & BridgeGraphicsPtr graphics don't need to be created as already exist in correct colours
    LinkGraphicsPtr[0] = gl1;
    LinkGraphicsPtr[1] = gl2;
    LinkGraphicsPtr[2] = gl6;
    LinkGraphicsPtr[3] = gl5;
    LinkGraphicsPtr[4] = gl3;
    LinkGraphicsPtr[5] = gl4;
    LinkGraphicsPtr[6] = gl22;
    LinkGraphicsPtr[7] = gl24;
    LinkGraphicsPtr[8] = gl21;
    LinkGraphicsPtr[9] = bm27;
    LinkGraphicsPtr[10] = gl25;
    LinkGraphicsPtr[11] = gl23;
    LinkGraphicsPtr[12] = gl26;
    LinkGraphicsPtr[13] = bm20;
    LinkGraphicsPtr[14] = gl19;
    LinkGraphicsPtr[15] = bm18;
    LinkGraphicsPtr[16] = gl64;
    LinkGraphicsPtr[17] = bm65;
    LinkGraphicsPtr[18] = gl66;
    LinkGraphicsPtr[19] = gl67;
    LinkGraphicsPtr[20] = gl80;
    LinkGraphicsPtr[21] = gl81;
    LinkGraphicsPtr[22] = gl82;
    LinkGraphicsPtr[23] = gl83;
    LinkGraphicsPtr[24] = gl84;
    LinkGraphicsPtr[25] = bm85;
    LinkGraphicsPtr[26] = gl86;
    LinkGraphicsPtr[27] = gl87;
    LinkGraphicsPtr[28] = UnderHFootbridge;
    LinkGraphicsPtr[29] = UnderVFootbridge;

    BridgeGraphicsPtr[0] = br1;
    BridgeGraphicsPtr[1] = br2;
    BridgeGraphicsPtr[2] = br3;
    BridgeGraphicsPtr[3] = br4;
    BridgeGraphicsPtr[4] = br5;
    BridgeGraphicsPtr[5] = br9;
    BridgeGraphicsPtr[6] = br10;
    BridgeGraphicsPtr[7] = br6;
    BridgeGraphicsPtr[8] = br7;
    BridgeGraphicsPtr[9] = br12;
    BridgeGraphicsPtr[10] = br8;
    BridgeGraphicsPtr[11] = br11;
	SetWebSafeHeadCodeGraphics(0);

	Modifier->attach_callback([this]() {this->loadGraphics();});
	Modifier->attach_callback([this](){this->loadSpeedButGlyphs();});
	Modifier->attach_callback([this](){this->ChangeAllTransparentColours();});
	Modifier->attach_callback([this](){this->SetUpAllDerivativeGraphics();});

}
// ---------------------------------------------------------------------------

TRailGraphics::~TRailGraphics()
{
	Modifier->clear_callbacks();

    for(int x = 0; x < 30; x++)
    {
        delete LinkPrefDirGraphicsPtr[x];
    }
    for(int x = 0; x < 30; x++)
    {
        delete LinkNonSigRouteGraphicsPtr[x];
    }
    for(int x = 0; x < 30; x++)
    {
        delete LinkSigRouteGraphicsPtr[x];
    }
    for(int x = 0; x < 30; x++)
    {
        delete LinkRouteAutoSigsGraphicsPtr[x];
    }
    for(int x = 0; x < 12; x++)
    {
        delete BridgePrefDirGraphicsPtr[x];
    }
    for(int x = 0; x < 12; x++)
    {
        delete BridgeNonSigRouteGraphicsPtr[x];
    }
    for(int x = 0; x < 12; x++)
    {
        delete BridgeSigRouteGraphicsPtr[x];
    }
    for(int x = 0; x < 12; x++)
    {
        delete BridgeRouteAutoSigsGraphicsPtr[x];
    }
    for(int x = 0; x < 10; x++)
    {
        delete DirectionPrefDirGraphicsPtr[x];
    }
    for(int x = 0; x < 10; x++)
    {
        delete DirectionNonSigRouteGraphicsPtr[x];
    }
    for(int x = 0; x < 10; x++)
    {
        delete DirectionSigRouteGraphicsPtr[x];
    }
    for(int x = 0; x < 10; x++)
    {
        delete DirectionRouteAutoSigsGraphicsPtr[x];
    }
    for(int x = 0; x < 32; x++)
    {
        delete PointModeGraphicsPtr[x][0];
        delete PointModeGraphicsPtr[x][1];
    }

    delete BlackCircle; //added at v2.13.0
    delete bm10;
    delete bm100;
    delete bm101;
    delete bm106;
    delete bm10Diverging;
    delete bm10Straight;
    delete bm11;
    delete bm11Diverging;
    delete bm11Straight;
    delete bm12;
    delete bm12Diverging;
    delete bm12Straight;
    delete bm13;
    delete bm132;
    delete bm132LeftFork;
    delete bm132RightFork;
    delete bm133;
    delete bm133LeftFork;
    delete bm133RightFork;
    delete bm134;
    delete bm134LeftFork;
    delete bm134RightFork;
    delete bm135;
    delete bm135LeftFork;
    delete bm135RightFork;
    delete bm136;
    delete bm136LeftFork;
    delete bm136RightFork;
    delete bm137;
    delete bm137LeftFork;
    delete bm137RightFork;
    delete bm138;
    delete bm138LeftFork;
    delete bm138RightFork;
    delete bm139;
    delete bm139LeftFork;
    delete bm139RightFork;
    delete bm13Diverging;
    delete bm13Straight;
    delete bm14;
    delete bm140;
    delete bm141;
    delete bm14Diverging;
    delete bm14Straight;
    delete bm16;
    delete bm18;
    delete bm20;
    delete bm27;
    delete bm28;
    delete bm28Diverging;
    delete bm28Straight;
    delete bm29;
    delete bm29Diverging;
    delete bm29Straight;
    delete bm30;
    delete bm30Diverging;
    delete bm30Straight;
    delete bm31;
    delete bm31Diverging;
    delete bm31Straight;
    delete bm32;
    delete bm32Diverging;
    delete bm32Straight;
    delete bm33;
    delete bm33Diverging;
    delete bm33Straight;
    delete bm34;
    delete bm34Diverging;
    delete bm34Straight;
    delete bm35;
    delete bm35Diverging;
    delete bm35Straight;
    delete bm36;
    delete bm36Diverging;
    delete bm36Straight;
    delete bm37;
    delete bm37Diverging;
    delete bm37Straight;
    delete bm38;
    delete bm38Diverging;
    delete bm38Straight;
    delete bm39;
    delete bm39Diverging;
    delete bm39Straight;
    delete bm40;
    delete bm40Diverging;
    delete bm40Straight;
    delete bm41;
    delete bm41Diverging;
    delete bm41Straight;
    delete bm42;
    delete bm42Diverging;
    delete bm42Straight;
    delete bm43;
    delete bm43Diverging;
    delete bm43Straight;
    delete bm45;
    delete bm46;
    delete bm50;
    delete bm51;
    delete bm53;
    delete bm54;
    delete bm56;
    delete bm59;
    delete bm65;
    delete bm68CallingOn;
    delete bm68dblyellow;
    delete bm68grounddblred; // added at v2.3.1 (missed)
    delete bm68grounddblwhite; // added at v2.3.1 (missed)
    delete bm68green;
    delete bm68yellow;
    delete bm69CallingOn;
    delete bm69dblyellow;
    delete bm69grounddblred; // added at v2.3.1 (missed)
    delete bm69grounddblwhite; // added at v2.3.1 (missed)
    delete bm69green;
    delete bm69yellow;
    delete bm7;
    delete bm70CallingOn;
    delete bm70dblyellow;
    delete bm70grounddblred; // added at v2.3.1 (missed)
    delete bm70grounddblwhite; // added at v2.3.1 (missed)
    delete bm70green;
    delete bm70yellow;
    delete bm71CallingOn;
    delete bm71dblyellow;
    delete bm71grounddblred; // added at v2.3.1 (missed)
    delete bm71grounddblwhite; // added at v2.3.1 (missed)
    delete bm71green;
    delete bm71yellow;
    delete bm72CallingOn;
    delete bm72dblyellow;
    delete bm72grounddblred; // added at v2.3.1 (missed)
    delete bm72grounddblwhite; // added at v2.3.1 (missed)
    delete bm72green;
    delete bm72yellow;
    delete bm73;
    delete bm73CallingOn;
    delete bm73dblyellow;
    delete bm73grounddblred; // added at v2.3.1 (missed)
    delete bm73grounddblwhite; // added at v2.3.1 (missed)
    delete bm73green;
    delete bm73yellow;
    delete bm74;
    delete bm74CallingOn;
    delete bm74dblyellow;
    delete bm74grounddblred; // added at v2.3.1 (missed)
    delete bm74grounddblwhite; // added at v2.3.1 (missed)
    delete bm74green;
    delete bm74yellow;
    delete bm75CallingOn;
    delete bm75dblyellow;
    delete bm75grounddblred; // added at v2.3.1 (missed)
    delete bm75grounddblwhite; // added at v2.3.1 (missed)
    delete bm75green;
    delete bm75yellow;
    delete bm77;
    delete bm77Striped;
    delete bm78;
    delete bm78Striped;
    delete bm7Diverging;
    delete bm7Straight;
    delete bm8;
    delete bm85;
    delete bm8Diverging;
    delete bm8Straight;
    delete bm9;
    delete bm93set;
    delete bm93unset;
    delete bm94set;
    delete bm94unset;
    delete bm9Diverging;
    delete bm9Straight;
    delete bmDiagonalSignalBlank;
    delete bmGreenEllipse;
    delete bmGreenRect;
    delete bmGrid;
    delete bmLightBlueRect;
    delete bmName;
    delete bmNameStriped;
    delete bmPointBlank;
    delete bmRedEllipse;
    delete bmRedRect;
    delete bmRtCancELink1;
    delete bmRtCancELink2;
    delete bmRtCancELink3;
    delete bmRtCancELink4;
    delete bmRtCancELink6;
    delete bmRtCancELink7;
    delete bmRtCancELink8;
    delete bmRtCancELink9;
    delete bmStraightEWSignalBlank;
    delete bmStraightNSSignalBlank;
    delete bmSolidBgnd;
    delete br1;
    delete br10;
    delete br11;
    delete br12;
    delete br2;
    delete br3;
    delete br4;
    delete br5;
    delete br6;
    delete br7;
    delete br8;
    delete br9;
    delete Code_a;
    delete Code_b;
    delete Code_c;
    delete Code_d;
    delete Code_e;
    delete Code_f;
    delete Code_g;
    delete Code_h;
    delete Code_i;
    delete Code_j;
    delete Code_k;
    delete Code_l;
    delete Code_m;
    delete Code_n;
    delete Code_o;
    delete Code_p;
    delete Code_q;
    delete Code_r;
    delete Code_s;
    delete Code_t;
    delete Code_u;
    delete Code_v;
    delete Code_w;
    delete Code_x;
    delete Code_y;
    delete Code_z;
    delete Code0;
    delete Code1;
    delete Code2;
    delete Code3;
    delete Code4;
    delete Code5;
    delete Code6;
    delete Code7;
    delete Code8;
    delete Code9;
    delete CodeA;
    delete CodeB;
    delete CodeC;
    delete CodeD;
    delete CodeE;
    delete CodeF;
    delete CodeG;
    delete CodeH;
    delete CodeI;
    delete CodeJ;
    delete CodeK;
    delete CodeL;
    delete CodeM;
    delete CodeN;
    delete CodeO;
    delete CodeP;
    delete CodeQ;
    delete CodeR;
    delete CodeS;
    delete CodeT;
    delete CodeU;
    delete CodeV;
    delete CodeW;
    delete CodeX;
    delete CodeY;
    delete CodeZ;
    delete Concourse;
    delete ConcourseGlyph;
    delete ConcourseStriped;

    delete CouplingExit1; //Multiplayer coupled exit graphics
    delete CouplingExit2;
    delete CouplingExit3;
    delete CouplingExit4;
    delete CouplingExit6;
    delete CouplingExit7;
    delete CouplingExit8;
    delete CouplingExit9;

    delete SolidCircleRed; //multiplayer panel images
    delete SolidCircleYellow;
    delete SolidCircleGreen;

    delete ELk1;
    delete ELk2;
    delete ELk3;
    delete ELk4;
    delete ELk6;
    delete ELk7;
    delete ELk8;
    delete ELk9;
    delete BlackOctagon;
    delete gl1;
    delete gl10;
    delete gl100;
    delete gl101;
    delete gl102;
    delete gl103;
    delete gl104;
    delete gl105;
    delete gl106;
    delete gl107;
    delete gl108;
    delete gl109;
    delete gl11;
    delete gl110;
    delete gl111;
    delete gl112;
    delete gl113;
    delete gl114;
    delete gl115;
    delete gl116;
    delete gl117;
    delete gl118;
    delete gl119;
    delete gl12;
    delete gl120;
    delete gl121;
    delete gl122;
    delete gl123;
    delete gl124;
    delete gl125;
    delete gl126;
    delete gl127;
    delete gl128;
    delete gl129;
    delete gl129Striped;
    delete gl13;
    delete gl130;
    delete gl130Striped;
    delete gl131;
    delete gl132;
    delete gl133;
    delete gl134;
    delete gl135;
    delete gl136;
    delete gl137;
    delete gl138;
    delete gl139;
    delete gl14;
    delete gl140;
    delete gl141;
    delete gl142;
    delete gl143;
    delete gl145;
    delete gl145Striped;
    delete gl146;
    delete gl146Striped;
    delete gl15;
    delete gl16;
    delete gl18;
    delete gl19;
    delete gl2;
    delete gl20;
    delete gl21;
    delete gl22;
    delete gl23;
    delete gl24;
    delete gl25;
    delete gl26;
    delete gl27;
    delete gl28;
    delete gl29;
    delete gl3;
    delete gl30;
    delete gl31;
    delete gl32;
    delete gl33;
    delete gl34;
    delete gl35;
    delete gl36;
    delete gl37;
    delete gl38;
    delete gl39;
    delete gl4;
    delete gl40;
    delete gl41;
    delete gl42;
    delete gl43;
    delete gl44;
    delete gl45;
    delete gl46;
    delete gl47;
    delete gl48;
    delete gl49;
    delete gl5;
    delete gl50;
    delete gl51;
    delete gl52;
    delete gl53;
    delete gl54;
    delete gl55;
    delete gl56;
    delete gl57;
    delete gl58;
    delete gl59;
    delete gl6;
    delete gl60;
    delete gl61;
    delete gl62;
    delete gl63;
    delete gl64;
    delete gl65;
    delete gl66;
    delete gl67;
    delete gl68;
    delete gl69;
    delete gl7;
    delete gl70;
    delete gl71;
    delete gl72;
    delete gl73;
    delete gl73grounddblred;
    delete gl74;
    delete gl74grounddblred;
    delete gl75;
    delete gl76;
    delete gl76Striped;
    delete gl77;
    delete gl78;
    delete gl79;
    delete gl79Striped;
    delete gl8;
    delete gl80;
    delete gl81;
    delete gl82;
    delete gl83;
    delete gl84;
    delete gl85;
    delete gl86;
    delete gl87;
    delete gl88set;
    delete gl88unset;
    delete gl89set;
    delete gl89unset;
    delete gl9;
    delete gl90set;
    delete gl90unset;
    delete gl91set;
    delete gl91unset;
    delete gl92set;
    delete gl92unset;
    delete gl93set;
    delete gl94set;
    delete gl95set;
    delete gl95unset;
    delete gl97;
    delete gl98;
    delete gl99;
    delete Plat68;
    delete Plat68Striped;
    delete Plat69;
    delete Plat69Striped;
    delete Plat70;
    delete Plat70Striped;
    delete Plat71;
    delete Plat71Striped;
    delete sm1;
    delete sm10;
    delete sm100;
    delete sm101;
    delete sm102;
    delete sm103;
    delete sm104;
    delete sm105;
    delete sm106;
    delete sm107;
    delete sm108;
    delete sm109;
    delete sm11;
    delete sm110;
    delete sm111;
    delete sm112;
    delete sm115;
    delete sm117;
    delete sm12;
    delete sm129;
    delete sm129striped;
    delete sm13;
    delete sm130;
    delete sm130striped;
    delete sm131striped;
    delete sm132;
    delete sm133;
    delete sm134;
    delete sm135;
    delete sm136;
    delete sm137;
    delete sm138;
    delete sm139;
    delete sm14;
    delete sm15;
    delete sm16;
    delete sm18;
    delete sm19;
    delete sm2;
    delete sm20;
    delete sm21;
    delete sm22;
    delete sm23;
    delete sm24;
    delete sm25;
    delete sm26;
    delete sm27;
    delete sm28;
    delete sm29;
    delete sm3;
    delete sm30;
    delete sm31;
    delete sm32;
    delete sm33;
    delete sm34;
    delete sm35;
    delete sm36;
    delete sm37;
    delete sm38;
    delete sm39;
    delete sm4;
    delete sm40;
    delete sm41;
    delete sm42;
    delete sm43;
    delete sm44;
    delete sm45;
    delete sm46;
    delete sm47;
    delete sm48;
    delete sm49;
    delete sm5;
    delete sm50;
    delete sm51;
    delete sm52;
    delete sm53;
    delete sm54;
    delete sm55;
    delete sm56;
    delete sm57;
    delete sm58;
    delete sm59;
    delete sm6;
    delete sm60;
    delete sm61;
    delete sm62;
    delete sm63;
    delete sm64;
    delete sm65;
    delete sm66;
    delete sm67;
    delete sm7;
    delete sm76;
    delete sm76striped;
    delete sm77;
    delete sm77striped;
    delete sm78;
    delete sm78striped;
    delete sm79;
    delete sm79striped;
    delete sm8;
    delete sm80;
    delete sm81;
    delete sm82;
    delete sm83;
    delete sm84;
    delete sm85;
    delete sm86;
    delete sm87;
    delete sm88;
    delete sm89;
    delete sm9;
    delete sm90;
    delete sm91;
    delete sm92;
    delete sm93;
    delete sm94;
    delete sm95;
    delete sm96;
    delete sm96striped;
    delete sm97;
    delete sm98;
    delete sm99;
    delete smBlack;
    delete smBlue;
    delete smBrightGreen;
    delete smCaramel;
    delete smCyan;
    delete smLC;
    delete smLightBlue;
    delete smMagenta;
    delete smName;
    delete smPaleGreen;
    delete smRed;
    delete smSolidBgnd;
    delete smYellow;
    delete smTransparent;
    delete TempBackground;
    delete TempHeadCode;
    delete UnderHFootbridge;
    delete UnderVFootbridge;

    delete LCBothHor;
    delete LCBotHor;
    delete LCBothVer;
    delete LCLHSVer;
    delete LCPlain;
    delete LCRHSVer;
    delete LCTopHor;
    delete LCBothHorMan;
    delete LCBotHorMan;
    delete LCBothVerMan;
    delete LCLHSVerMan;
    delete LCPlainMan;
    delete LCRHSVerMan;
    delete LCTopHorMan;

    delete FSig68; //failed signals added at v2.13.0
    delete FSig69;
    delete FSig70;
    delete FSig71;
    delete FSig72;
    delete FSig73;
    delete FSig74;
    delete FSig75;
    delete FGSig68;
    delete FGSig69;
    delete FGSig70;
    delete FGSig71;
    delete FGSig72;
    delete FGSig73;
    delete FGSig74;
    delete FGSig75;

    delete SpeedBut68NormBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut69NormBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut70NormBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut71NormBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut72NormBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut73NormBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut74NormBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut75NormBlackGlyph; // added at v2.3.1 (missed)

    delete SpeedBut68GrndBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut69GrndBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut70GrndBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut71GrndBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut72GrndBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut73GrndBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut74GrndBlackGlyph; // added at v2.3.1 (missed)
    delete SpeedBut75GrndBlackGlyph; // added at v2.3.1 (missed)

    delete bmTransparentBgnd;
    delete GridBitmap;

    delete HeatMapGraphic; //added at v2.22.0
}

// ---------------------------------------------------------------------------

void TRailGraphics::SetWebSafePalette(int Caller, Graphics::TBitmap *bmp)  //generated by ChatGPT
{
    Utilities->CallLog.push_back(Utilities->TimeStamp() + ",SetWebSafePalette");
    LOGPALETTE *logPal;
    logPal = (LOGPALETTE*) new char[sizeof(LOGPALETTE) + 255 * sizeof(PALETTEENTRY)];
    logPal->palVersion = 0x300;
    logPal->palNumEntries = 219; // Web-safe palette has 216 colors + 3 extra

    // Generate Web-Safe Colors
    int index = 0;
    for (int r = 0; r <= 255; r += 51) {
        for (int g = 0; g <= 255; g += 51) {
            for (int b = 0; b <= 255; b += 51) {
                logPal->palPalEntry[index].peRed = r;
                logPal->palPalEntry[index].peGreen = g;
                logPal->palPalEntry[index].peBlue = b;
                logPal->palPalEntry[index].peFlags = PC_NOCOLLAPSE;
                index++;
            }
        }
    }
// add near transparent colours:
//216 = near black = B005; G0, R0
//217 = near dark blue = B046; G0, R0
//218 = nerar white = B250, G255, R255
    logPal->palPalEntry[216].peBlue = 5;  // }
    logPal->palPalEntry[216].peGreen = 0; // } near black
    logPal->palPalEntry[216].peRed = 0;   // }

    logPal->palPalEntry[217].peBlue = 46; // }
    logPal->palPalEntry[217].peGreen = 0; // } near dark blue
    logPal->palPalEntry[217].peRed = 0;   // }

    logPal->palPalEntry[218].peBlue = 250;  // }
    logPal->palPalEntry[218].peGreen = 255; // } near white
    logPal->palPalEntry[218].peRed = 255;   // }

    // Create and assign the palette
    HPALETTE hPal = CreatePalette(logPal);
    delete[] logPal; // Free memory

    bmp->Palette = hPal; // Assign the custom web-safe palette
    Utilities->CallLogPop(2715);
}

// ---------------------------------------------------------------------------

void TRailGraphics::SetWebSafeHeadCodeGraphics(int Caller)
{
    Utilities->CallLog.push_back(Utilities->TimeStamp() + ",SetWebSafeHeadCodeGraphics");
    SetWebSafePalette(0, Code_a);
    SetWebSafePalette(1, Code_b);
    SetWebSafePalette(2, Code_c);
    SetWebSafePalette(3, Code_d);
    SetWebSafePalette(4, Code_e);
    SetWebSafePalette(5, Code_f);
    SetWebSafePalette(6, Code_g);
    SetWebSafePalette(7, Code_h);
    SetWebSafePalette(8, Code_i);
    SetWebSafePalette(9, Code_j);
    SetWebSafePalette(10, Code_k);
    SetWebSafePalette(11, Code_l);
    SetWebSafePalette(12, Code_m);
    SetWebSafePalette(13, Code_n);
    SetWebSafePalette(14, Code_o);
    SetWebSafePalette(15, Code_p);
    SetWebSafePalette(16, Code_q);
    SetWebSafePalette(17, Code_r);
    SetWebSafePalette(18, Code_s);
    SetWebSafePalette(19, Code_t);
    SetWebSafePalette(20, Code_u);
    SetWebSafePalette(21, Code_v);
    SetWebSafePalette(22, Code_w);
    SetWebSafePalette(23, Code_x);
    SetWebSafePalette(24, Code_y);
    SetWebSafePalette(25, Code_z);
    SetWebSafePalette(26, Code0);
    SetWebSafePalette(27, Code1);
    SetWebSafePalette(28, Code2);
    SetWebSafePalette(29, Code3);
    SetWebSafePalette(30, Code4);
    SetWebSafePalette(31, Code5);
    SetWebSafePalette(32, Code6);
    SetWebSafePalette(33, Code7);
    SetWebSafePalette(34, Code8);
    SetWebSafePalette(35, Code9);
    SetWebSafePalette(36, CodeA);
    SetWebSafePalette(37, CodeB);
    SetWebSafePalette(38, CodeC);
    SetWebSafePalette(39, CodeD);
    SetWebSafePalette(40, CodeE);
    SetWebSafePalette(41, CodeF);
    SetWebSafePalette(42, CodeG);
    SetWebSafePalette(43, CodeH);
    SetWebSafePalette(44, CodeI);
    SetWebSafePalette(45, CodeJ);
    SetWebSafePalette(46, CodeK);
    SetWebSafePalette(47, CodeL);
    SetWebSafePalette(48, CodeM);
    SetWebSafePalette(49, CodeN);
    SetWebSafePalette(50, CodeO);
    SetWebSafePalette(51, CodeP);
    SetWebSafePalette(52, CodeQ);
    SetWebSafePalette(53, CodeR);
    SetWebSafePalette(54, CodeS);
    SetWebSafePalette(55, CodeT);
    SetWebSafePalette(56, CodeU);
    SetWebSafePalette(57, CodeV);
    SetWebSafePalette(58, CodeW);
    SetWebSafePalette(59, CodeX);
    SetWebSafePalette(60, CodeY);
    SetWebSafePalette(61, CodeZ);
    Utilities->CallLogPop(2716);
}

// ---------------------------------------------------------------------------

void TRailGraphics::loadGraphics() {
	// transparent graphics
	Modifier->load_graphic(bm10, "bm10", Transparency::Transparent);
	Modifier->load_graphic(BlackCircle, "BlackCircle", Transparency::Transparent);
    Modifier->load_graphic(bm100, "bm100", Transparency::Transparent);
    Modifier->load_graphic(bm101, "bm101", Transparency::Transparent);
    Modifier->load_graphic(bm106, "bm106", Transparency::Transparent);
    Modifier->load_graphic(bm10Diverging, "bm10Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm10Straight, "bm10Straight", Transparency::Transparent);
    Modifier->load_graphic(bm11, "bm11", Transparency::Transparent);
    Modifier->load_graphic(bm11Diverging, "bm11Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm11Straight, "bm11Straight", Transparency::Transparent);
    Modifier->load_graphic(bm12, "bm12", Transparency::Transparent);
    Modifier->load_graphic(bm12Diverging, "bm12Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm12Straight, "bm12Straight", Transparency::Transparent);
    Modifier->load_graphic(bm13, "bm13", Transparency::Transparent);
    Modifier->load_graphic(bm132, "bm132", Transparency::Transparent);
    Modifier->load_graphic(bm132LeftFork, "bm132LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm132RightFork, "bm132RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm133, "bm133", Transparency::Transparent);
    Modifier->load_graphic(bm133LeftFork, "bm133LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm133RightFork, "bm133RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm134, "bm134", Transparency::Transparent);
    Modifier->load_graphic(bm134LeftFork, "bm134LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm134RightFork, "bm134RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm135, "bm135", Transparency::Transparent);
    Modifier->load_graphic(bm135LeftFork, "bm135LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm135RightFork, "bm135RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm136, "bm136", Transparency::Transparent);
    Modifier->load_graphic(bm136LeftFork, "bm136LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm136RightFork, "bm136RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm137, "bm137", Transparency::Transparent);
    Modifier->load_graphic(bm137LeftFork, "bm137LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm137RightFork, "bm137RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm138, "bm138", Transparency::Transparent);
    Modifier->load_graphic(bm138LeftFork, "bm138LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm138RightFork, "bm138RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm139, "bm139", Transparency::Transparent);
    Modifier->load_graphic(bm139LeftFork, "bm139LeftFork", Transparency::Transparent);
    Modifier->load_graphic(bm139RightFork, "bm139RightFork", Transparency::Transparent);
    Modifier->load_graphic(bm13Diverging, "bm13Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm13Straight, "bm13Straight", Transparency::Transparent);
    Modifier->load_graphic(bm14, "bm14", Transparency::Transparent);
    Modifier->load_graphic(bm140, "bm140", Transparency::Transparent);
    Modifier->load_graphic(bm141, "bm141", Transparency::Transparent);
    Modifier->load_graphic(bm14Diverging, "bm14Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm14Straight, "bm14Straight", Transparency::Transparent);
    Modifier->load_graphic(bm16, "bm16", Transparency::Transparent);
    Modifier->load_graphic(bm18, "bm18", Transparency::Transparent);
    Modifier->load_graphic(bm20, "bm20", Transparency::Transparent);
    Modifier->load_graphic(bm27, "bm27", Transparency::Transparent);
    Modifier->load_graphic(bm28, "bm28", Transparency::Transparent);
    Modifier->load_graphic(bm28Diverging, "bm28Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm28Straight, "bm28Straight", Transparency::Transparent);
    Modifier->load_graphic(bm29, "bm29", Transparency::Transparent);
    Modifier->load_graphic(bm29Diverging, "bm29Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm29Straight, "bm29Straight", Transparency::Transparent);
    Modifier->load_graphic(bm30, "bm30", Transparency::Transparent);
    Modifier->load_graphic(bm30Diverging, "bm30Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm30Straight, "bm30Straight", Transparency::Transparent);
    Modifier->load_graphic(bm31, "bm31", Transparency::Transparent);
    Modifier->load_graphic(bm31Diverging, "bm31Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm31Straight, "bm31Straight", Transparency::Transparent);
    Modifier->load_graphic(bm32, "bm32", Transparency::Transparent);
    Modifier->load_graphic(bm32Diverging, "bm32Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm32Straight, "bm32Straight", Transparency::Transparent);
    Modifier->load_graphic(bm33, "bm33", Transparency::Transparent);
    Modifier->load_graphic(bm33Diverging, "bm33Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm33Straight, "bm33Straight", Transparency::Transparent);
    Modifier->load_graphic(bm34, "bm34", Transparency::Transparent);
    Modifier->load_graphic(bm34Diverging, "bm34Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm34Straight, "bm34Straight", Transparency::Transparent);
    Modifier->load_graphic(bm35, "bm35", Transparency::Transparent);
    Modifier->load_graphic(bm35Diverging, "bm35Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm35Straight, "bm35Straight", Transparency::Transparent);
    Modifier->load_graphic(bm36, "bm36", Transparency::Transparent);
    Modifier->load_graphic(bm36Diverging, "bm36Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm36Straight, "bm36Straight", Transparency::Transparent);
    Modifier->load_graphic(bm37, "bm37", Transparency::Transparent);
    Modifier->load_graphic(bm37Diverging, "bm37Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm37Straight, "bm37Straight", Transparency::Transparent);
    Modifier->load_graphic(bm38, "bm38", Transparency::Transparent);
    Modifier->load_graphic(bm38Diverging, "bm38Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm38Straight, "bm38Straight", Transparency::Transparent);
    Modifier->load_graphic(bm39, "bm39", Transparency::Transparent);
    Modifier->load_graphic(bm39Diverging, "bm39Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm39Straight, "bm39Straight", Transparency::Transparent);
    Modifier->load_graphic(bm40, "bm40", Transparency::Transparent);
    Modifier->load_graphic(bm40Diverging, "bm40Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm40Straight, "bm40Straight", Transparency::Transparent);
    Modifier->load_graphic(bm41, "bm41", Transparency::Transparent);
    Modifier->load_graphic(bm41Diverging, "bm41Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm41Straight, "bm41Straight", Transparency::Transparent);
    Modifier->load_graphic(bm42, "bm42", Transparency::Transparent);
    Modifier->load_graphic(bm42Diverging, "bm42Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm42Straight, "bm42Straight", Transparency::Transparent);
    Modifier->load_graphic(bm43, "bm43", Transparency::Transparent);
    Modifier->load_graphic(bm43Diverging, "bm43Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm43Straight, "bm43Straight", Transparency::Transparent);
    Modifier->load_graphic(bm45, "bm45", Transparency::Transparent);
    Modifier->load_graphic(bm46, "bm46", Transparency::Transparent);
    Modifier->load_graphic(bm50, "bm50", Transparency::Transparent);
    Modifier->load_graphic(bm51, "bm51", Transparency::Transparent);
    Modifier->load_graphic(bm53, "bm53", Transparency::Transparent);
    Modifier->load_graphic(bm54, "bm54", Transparency::Transparent);
    Modifier->load_graphic(bm56, "bm56", Transparency::Transparent);
    Modifier->load_graphic(bm59, "bm59", Transparency::Transparent);
    Modifier->load_graphic(bm65, "bm65", Transparency::Transparent);
    Modifier->load_graphic(bm68CallingOn, "bm68CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm68dblyellow, "bm68dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm68grounddblred, "bm68grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm68grounddblwhite, "bm68grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm68green, "bm68green", Transparency::Transparent);
    Modifier->load_graphic(bm68yellow, "bm68yellow", Transparency::Transparent);
    Modifier->load_graphic(bm69CallingOn, "bm69CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm69dblyellow, "bm69dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm69grounddblred, "bm69grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm69grounddblwhite, "bm69grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm69green, "bm69green", Transparency::Transparent);
    Modifier->load_graphic(bm69yellow, "bm69yellow", Transparency::Transparent);
    Modifier->load_graphic(bm7, "bm7", Transparency::Transparent);
    Modifier->load_graphic(bm70CallingOn, "bm70CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm70dblyellow, "bm70dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm70grounddblred, "bm70grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm70grounddblwhite, "bm70grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm70green, "bm70green", Transparency::Transparent);
    Modifier->load_graphic(bm70yellow, "bm70yellow", Transparency::Transparent);
    Modifier->load_graphic(bm71CallingOn, "bm71CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm71dblyellow, "bm71dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm71grounddblred, "bm71grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm71grounddblwhite, "bm71grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm71green, "bm71green", Transparency::Transparent);
    Modifier->load_graphic(bm71yellow, "bm71yellow", Transparency::Transparent);
    Modifier->load_graphic(bm72CallingOn, "bm72CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm72dblyellow, "bm72dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm72grounddblred, "bm72grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm72grounddblwhite, "bm72grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm72green, "bm72green", Transparency::Transparent);
    Modifier->load_graphic(bm72yellow, "bm72yellow", Transparency::Transparent);
    Modifier->load_graphic(bm73, "bm73", Transparency::Transparent);
    Modifier->load_graphic(bm73CallingOn, "bm73CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm73dblyellow, "bm73dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm73grounddblred, "bm73grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm73grounddblwhite, "bm73grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm73green, "bm73green", Transparency::Transparent);
    Modifier->load_graphic(bm73yellow, "bm73yellow", Transparency::Transparent);
    Modifier->load_graphic(bm74, "bm74", Transparency::Transparent);
    Modifier->load_graphic(bm74CallingOn, "bm74CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm74dblyellow, "bm74dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm74grounddblred, "bm74grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm74grounddblwhite, "bm74grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm74green, "bm74green", Transparency::Transparent);
    Modifier->load_graphic(bm74yellow, "bm74yellow", Transparency::Transparent);
    Modifier->load_graphic(bm75CallingOn, "bm75CallingOn", Transparency::Transparent);
    Modifier->load_graphic(bm75dblyellow, "bm75dblyellow", Transparency::Transparent);
    Modifier->load_graphic(bm75grounddblred, "bm75grounddblred", Transparency::Transparent);
    Modifier->load_graphic(bm75grounddblwhite, "bm75grounddblwhite", Transparency::Transparent);
    Modifier->load_graphic(bm75green, "bm75green", Transparency::Transparent);
    Modifier->load_graphic(bm75yellow, "bm75yellow", Transparency::Transparent);
    Modifier->load_graphic(bm77, "bm77", Transparency::Transparent);
    Modifier->load_graphic(bm77Striped, "bm77Striped", Transparency::Transparent);
    Modifier->load_graphic(bm78, "bm78", Transparency::Transparent);
    Modifier->load_graphic(bm78Striped, "bm78Striped", Transparency::Transparent);
    Modifier->load_graphic(bm7Diverging, "bm7Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm7Straight, "bm7Straight", Transparency::Transparent);
    Modifier->load_graphic(bm8, "bm8", Transparency::Transparent);
    Modifier->load_graphic(bm85, "bm85", Transparency::Transparent);
    Modifier->load_graphic(bm8Diverging, "bm8Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm8Straight, "bm8Straight", Transparency::Transparent);
    Modifier->load_graphic(bm9, "bm9", Transparency::Transparent);
    Modifier->load_graphic(bm93set, "bm93set", Transparency::Transparent);
    Modifier->load_graphic(bm93unset, "bm93unset", Transparency::Transparent);
    Modifier->load_graphic(bm94set, "bm94set", Transparency::Transparent);
    Modifier->load_graphic(bm94unset, "bm94unset", Transparency::Transparent);
    Modifier->load_graphic(bm9Diverging, "bm9Diverging", Transparency::Transparent);
    Modifier->load_graphic(bm9Straight, "bm9Straight", Transparency::Transparent);
    Modifier->load_graphic(bmGreenEllipse, "bmGreenEllipse", Transparency::Transparent);
    Modifier->load_graphic(bmGreenRect, "bmGreenRect", Transparency::Transparent);
    Modifier->load_graphic(bmGrid, "bmGrid", Transparency::Transparent);
    Modifier->load_graphic(bmLightBlueRect, "bmLightBlueRect", Transparency::Transparent);
    Modifier->load_graphic(bmName, "bmName", Transparency::Transparent);
    Modifier->load_graphic(bmNameStriped, "bmNameStriped", Transparency::Transparent);
    Modifier->load_graphic(bmRedEllipse, "bmRedEllipse", Transparency::Transparent);
    Modifier->load_graphic(bmRedRect, "bmRedRect", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink1, "bmRtCancELink1", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink2, "bmRtCancELink2", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink3, "bmRtCancELink3", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink4, "bmRtCancELink4", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink6, "bmRtCancELink6", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink7, "bmRtCancELink7", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink8, "bmRtCancELink8", Transparency::Transparent);
    Modifier->load_graphic(bmRtCancELink9, "bmRtCancELink9", Transparency::Transparent);
    Modifier->load_graphic(br1, "br1", Transparency::Transparent);
    Modifier->load_graphic(br10, "br10", Transparency::Transparent);
    Modifier->load_graphic(br11, "br11", Transparency::Transparent);
    Modifier->load_graphic(br12, "br12", Transparency::Transparent);
    Modifier->load_graphic(br2, "br2", Transparency::Transparent);
    Modifier->load_graphic(br3, "br3", Transparency::Transparent);
    Modifier->load_graphic(br4, "br4", Transparency::Transparent);
    Modifier->load_graphic(br5, "br5", Transparency::Transparent);
    Modifier->load_graphic(br6, "br6", Transparency::Transparent);
    Modifier->load_graphic(br7, "br7", Transparency::Transparent);
    Modifier->load_graphic(br8, "br8", Transparency::Transparent);
    Modifier->load_graphic(br9, "br9", Transparency::Transparent);
    Modifier->load_graphic(Concourse, "Concourse", Transparency::Transparent);
    Modifier->load_graphic(ConcourseGlyph, "ConcourseGlyph", Transparency::Transparent);
    Modifier->load_graphic(ConcourseStriped, "ConcourseStriped", Transparency::Transparent);

	Modifier->load_graphic(CouplingExit1, "CouplingExit1", Transparency::Transparent);
    Modifier->load_graphic(CouplingExit2, "CouplingExit2", Transparency::Transparent);
    Modifier->load_graphic(CouplingExit3, "CouplingExit3", Transparency::Transparent);
    Modifier->load_graphic(CouplingExit4, "CouplingExit4", Transparency::Transparent);
    Modifier->load_graphic(CouplingExit6, "CouplingExit6", Transparency::Transparent);
    Modifier->load_graphic(CouplingExit7, "CouplingExit7", Transparency::Transparent);
    Modifier->load_graphic(CouplingExit8, "CouplingExit8", Transparency::Transparent);
	Modifier->load_graphic(CouplingExit9, "CouplingExit9", Transparency::Transparent);

	Modifier->load_graphic(SolidCircleRed, "SolidCircleRed", Transparency::Transparent);
	Modifier->load_graphic(SolidCircleYellow, "SolidCircleYellow", Transparency::Transparent);
	Modifier->load_graphic(SolidCircleGreen, "SolidCircleGreen", Transparency::Transparent);

	Modifier->load_graphic(ELk1, "ELk1", Transparency::Transparent);
	Modifier->load_graphic(ELk2, "ELk2", Transparency::Transparent);
	Modifier->load_graphic(ELk3, "ELk3", Transparency::Transparent);
	Modifier->load_graphic(ELk4, "ELk4", Transparency::Transparent);
	Modifier->load_graphic(ELk6, "ELk6", Transparency::Transparent);
	Modifier->load_graphic(ELk7, "ELk7", Transparency::Transparent);
	Modifier->load_graphic(ELk8, "ELk8", Transparency::Transparent);
	Modifier->load_graphic(ELk9, "ELk9", Transparency::Transparent);
	Modifier->load_graphic(BlackOctagon, "BlackOctagon", Transparency::Transparent);
	Modifier->load_graphic(gl1, "gl1", Transparency::Transparent);
	Modifier->load_graphic(gl10, "gl10", Transparency::Transparent);
	Modifier->load_graphic(gl100, "gl100", Transparency::Transparent);
	Modifier->load_graphic(gl101, "gl101", Transparency::Transparent);
	Modifier->load_graphic(gl102, "gl102", Transparency::Transparent);
	Modifier->load_graphic(gl103, "gl103", Transparency::Transparent);
	Modifier->load_graphic(gl104, "gl104", Transparency::Transparent);
	Modifier->load_graphic(gl105, "gl105", Transparency::Transparent);
	Modifier->load_graphic(gl106, "gl106", Transparency::Transparent);
    Modifier->load_graphic(gl107, "gl107", Transparency::Transparent);
    Modifier->load_graphic(gl108, "gl108", Transparency::Transparent);
    Modifier->load_graphic(gl109, "gl109", Transparency::Transparent);
    Modifier->load_graphic(gl11, "gl11", Transparency::Transparent);
    Modifier->load_graphic(gl110, "gl110", Transparency::Transparent);
	Modifier->load_graphic(gl111, "gl111", Transparency::Transparent);
    Modifier->load_graphic(gl112, "gl112", Transparency::Transparent);
    Modifier->load_graphic(gl113, "gl113", Transparency::Transparent);
    Modifier->load_graphic(gl114, "gl114", Transparency::Transparent);
    Modifier->load_graphic(gl115, "gl115", Transparency::Transparent);
    Modifier->load_graphic(gl116, "gl116", Transparency::Transparent);
    Modifier->load_graphic(gl117, "gl117", Transparency::Transparent);
    Modifier->load_graphic(gl118, "gl118", Transparency::Transparent);
    Modifier->load_graphic(gl119, "gl119", Transparency::Transparent);
	Modifier->load_graphic(gl12, "gl12", Transparency::Transparent);
    Modifier->load_graphic(gl120, "gl120", Transparency::Transparent);
    Modifier->load_graphic(gl121, "gl121", Transparency::Transparent);
    Modifier->load_graphic(gl122, "gl122", Transparency::Transparent);
    Modifier->load_graphic(gl123, "gl123", Transparency::Transparent);
    Modifier->load_graphic(gl124, "gl124", Transparency::Transparent);
    Modifier->load_graphic(gl125, "gl125", Transparency::Transparent);
    Modifier->load_graphic(gl126, "gl126", Transparency::Transparent);
    Modifier->load_graphic(gl127, "gl127", Transparency::Transparent);
    Modifier->load_graphic(gl128, "gl128", Transparency::Transparent);
    Modifier->load_graphic(gl129, "gl129", Transparency::Transparent);
    Modifier->load_graphic(gl129Striped, "gl129Striped", Transparency::Transparent);
    Modifier->load_graphic(gl13, "gl13", Transparency::Transparent);
    Modifier->load_graphic(gl130, "gl130", Transparency::Transparent);
    Modifier->load_graphic(gl130Striped, "gl130Striped", Transparency::Transparent);
    Modifier->load_graphic(gl131, "gl131", Transparency::Transparent);
    Modifier->load_graphic(gl132, "gl132", Transparency::Transparent);
    Modifier->load_graphic(gl133, "gl133", Transparency::Transparent);
    Modifier->load_graphic(gl134, "gl134", Transparency::Transparent);
    Modifier->load_graphic(gl135, "gl135", Transparency::Transparent);
    Modifier->load_graphic(gl136, "gl136", Transparency::Transparent);
	Modifier->load_graphic(gl137, "gl137", Transparency::Transparent);
	Modifier->load_graphic(gl138, "gl138", Transparency::Transparent);
    Modifier->load_graphic(gl139, "gl139", Transparency::Transparent);
    Modifier->load_graphic(gl14, "gl14", Transparency::Transparent);
    Modifier->load_graphic(gl140, "gl140", Transparency::Transparent);
	Modifier->load_graphic(gl141, "gl141", Transparency::Transparent);
    Modifier->load_graphic(gl142, "gl142", Transparency::Transparent);
    Modifier->load_graphic(gl143, "gl143", Transparency::Transparent);
    Modifier->load_graphic(gl145, "gl145", Transparency::Transparent);
	Modifier->load_graphic(gl145Striped, "gl145Striped", Transparency::Transparent);
    Modifier->load_graphic(gl146, "gl146", Transparency::Transparent);
    Modifier->load_graphic(gl146Striped, "gl146Striped", Transparency::Transparent);
    Modifier->load_graphic(gl15, "gl15", Transparency::Transparent);
	Modifier->load_graphic(gl16, "gl16", Transparency::Transparent);
    Modifier->load_graphic(gl18, "gl18", Transparency::Transparent);
    Modifier->load_graphic(gl19, "gl19", Transparency::Transparent);
    Modifier->load_graphic(gl2, "gl2", Transparency::Transparent);
	Modifier->load_graphic(gl20, "gl20", Transparency::Transparent);
    Modifier->load_graphic(gl21, "gl21", Transparency::Transparent);
    Modifier->load_graphic(gl22, "gl22", Transparency::Transparent);
    Modifier->load_graphic(gl23, "gl23", Transparency::Transparent);
	Modifier->load_graphic(gl24, "gl24", Transparency::Transparent);
    Modifier->load_graphic(gl25, "gl25", Transparency::Transparent);
    Modifier->load_graphic(gl26, "gl26", Transparency::Transparent);
    Modifier->load_graphic(gl27, "gl27", Transparency::Transparent);
	Modifier->load_graphic(gl28, "gl28", Transparency::Transparent);
    Modifier->load_graphic(gl29, "gl29", Transparency::Transparent);
    Modifier->load_graphic(gl3, "gl3", Transparency::Transparent);
    Modifier->load_graphic(gl30, "gl30", Transparency::Transparent);
	Modifier->load_graphic(gl31, "gl31", Transparency::Transparent);
    Modifier->load_graphic(gl32, "gl32", Transparency::Transparent);
    Modifier->load_graphic(gl33, "gl33", Transparency::Transparent);
    Modifier->load_graphic(gl34, "gl34", Transparency::Transparent);
	Modifier->load_graphic(gl35, "gl35", Transparency::Transparent);
    Modifier->load_graphic(gl36, "gl36", Transparency::Transparent);
    Modifier->load_graphic(gl37, "gl37", Transparency::Transparent);
    Modifier->load_graphic(gl38, "gl38", Transparency::Transparent);
	Modifier->load_graphic(gl39, "gl39", Transparency::Transparent);
	Modifier->load_graphic(gl4, "gl4", Transparency::Transparent);
    Modifier->load_graphic(gl40, "gl40", Transparency::Transparent);
    Modifier->load_graphic(gl41, "gl41", Transparency::Transparent);
    Modifier->load_graphic(gl42, "gl42", Transparency::Transparent);
    Modifier->load_graphic(gl43, "gl43", Transparency::Transparent);
    Modifier->load_graphic(gl44, "gl44", Transparency::Transparent);
    Modifier->load_graphic(gl45, "gl45", Transparency::Transparent);
    Modifier->load_graphic(gl46, "gl46", Transparency::Transparent);
    Modifier->load_graphic(gl47, "gl47", Transparency::Transparent);
    Modifier->load_graphic(gl48, "gl48", Transparency::Transparent);
    Modifier->load_graphic(gl49, "gl49", Transparency::Transparent);
    Modifier->load_graphic(gl5, "gl5", Transparency::Transparent);
    Modifier->load_graphic(gl50, "gl50", Transparency::Transparent);
    Modifier->load_graphic(gl51, "gl51", Transparency::Transparent);
    Modifier->load_graphic(gl52, "gl52", Transparency::Transparent);
    Modifier->load_graphic(gl53, "gl53", Transparency::Transparent);
    Modifier->load_graphic(gl54, "gl54", Transparency::Transparent);
    Modifier->load_graphic(gl55, "gl55", Transparency::Transparent);
    Modifier->load_graphic(gl56, "gl56", Transparency::Transparent);
    Modifier->load_graphic(gl57, "gl57", Transparency::Transparent);
    Modifier->load_graphic(gl58, "gl58", Transparency::Transparent);
    Modifier->load_graphic(gl59, "gl59", Transparency::Transparent);
    Modifier->load_graphic(gl6, "gl6", Transparency::Transparent);
    Modifier->load_graphic(gl60, "gl60", Transparency::Transparent);
    Modifier->load_graphic(gl61, "gl61", Transparency::Transparent);
    Modifier->load_graphic(gl62, "gl62", Transparency::Transparent);
    Modifier->load_graphic(gl63, "gl63", Transparency::Transparent);
    Modifier->load_graphic(gl64, "gl64", Transparency::Transparent);
    Modifier->load_graphic(gl65, "gl65", Transparency::Transparent);
	Modifier->load_graphic(gl66, "gl66", Transparency::Transparent);
    Modifier->load_graphic(gl67, "gl67", Transparency::Transparent);
	Modifier->load_graphic(gl68, "gl68", Transparency::Transparent);
	Modifier->load_graphic(gl69, "gl69", Transparency::Transparent);
    Modifier->load_graphic(gl7, "gl7", Transparency::Transparent);
	Modifier->load_graphic(gl70, "gl70", Transparency::Transparent);
    Modifier->load_graphic(gl71, "gl71", Transparency::Transparent);
    Modifier->load_graphic(gl72, "gl72", Transparency::Transparent);
    Modifier->load_graphic(gl73, "gl73", Transparency::Transparent);
    Modifier->load_graphic(gl73grounddblred, "gl73grounddblred", Transparency::Transparent);
	Modifier->load_graphic(gl74, "gl74", Transparency::Transparent);
    Modifier->load_graphic(gl74grounddblred, "gl74grounddblred", Transparency::Transparent);
    Modifier->load_graphic(gl75, "gl75", Transparency::Transparent);
    Modifier->load_graphic(gl76, "gl76", Transparency::Transparent);
    Modifier->load_graphic(gl76Striped, "gl76Striped", Transparency::Transparent);
	Modifier->load_graphic(gl77, "gl77", Transparency::Transparent);
    Modifier->load_graphic(gl78, "gl78", Transparency::Transparent);
    Modifier->load_graphic(gl79, "gl79", Transparency::Transparent);
    Modifier->load_graphic(gl79Striped, "gl79Striped", Transparency::Transparent);
    Modifier->load_graphic(gl8, "gl8", Transparency::Transparent);
	Modifier->load_graphic(gl80, "gl80", Transparency::Transparent);
    Modifier->load_graphic(gl81, "gl81", Transparency::Transparent);
    Modifier->load_graphic(gl82, "gl82", Transparency::Transparent);
    Modifier->load_graphic(gl83, "gl83", Transparency::Transparent);
    Modifier->load_graphic(gl84, "gl84", Transparency::Transparent);
	Modifier->load_graphic(gl85, "gl85", Transparency::Transparent);
    Modifier->load_graphic(gl86, "gl86", Transparency::Transparent);
    Modifier->load_graphic(gl87, "gl87", Transparency::Transparent);
    Modifier->load_graphic(gl88set, "gl88set", Transparency::Transparent);
    Modifier->load_graphic(gl88unset, "gl88unset", Transparency::Transparent);
	Modifier->load_graphic(gl89set, "gl89set", Transparency::Transparent);
    Modifier->load_graphic(gl89unset, "gl89unset", Transparency::Transparent);
    Modifier->load_graphic(gl9, "gl9", Transparency::Transparent);
    Modifier->load_graphic(gl90set, "gl90set", Transparency::Transparent);
    Modifier->load_graphic(gl90unset, "gl90unset", Transparency::Transparent);
	Modifier->load_graphic(gl91set, "gl91set", Transparency::Transparent);
    Modifier->load_graphic(gl91unset, "gl91unset", Transparency::Transparent);
    Modifier->load_graphic(gl92set, "gl92set", Transparency::Transparent);
    Modifier->load_graphic(gl92unset, "gl92unset", Transparency::Transparent);
    Modifier->load_graphic(gl93set, "gl93set", Transparency::Transparent);
	Modifier->load_graphic(gl94set, "gl94set", Transparency::Transparent);
    Modifier->load_graphic(gl95set, "gl95set", Transparency::Transparent);
    Modifier->load_graphic(gl95unset, "gl95unset", Transparency::Transparent);
    Modifier->load_graphic(gl97, "gl97", Transparency::Transparent);
    Modifier->load_graphic(gl98, "gl98", Transparency::Transparent);
	Modifier->load_graphic(gl99, "gl99", Transparency::Transparent);
    Modifier->load_graphic(Plat68, "Plat68", Transparency::Transparent);
    Modifier->load_graphic(Plat68Striped, "Plat68Striped", Transparency::Transparent);
    Modifier->load_graphic(Plat69, "Plat69", Transparency::Transparent);
    Modifier->load_graphic(Plat69Striped, "Plat69Striped", Transparency::Transparent);
	Modifier->load_graphic(Plat70, "Plat70", Transparency::Transparent);
    Modifier->load_graphic(Plat70Striped, "Plat70Striped", Transparency::Transparent);
    Modifier->load_graphic(Plat71, "Plat71", Transparency::Transparent);
    Modifier->load_graphic(Plat71Striped, "Plat71Striped", Transparency::Transparent);
    Modifier->load_graphic(sm1, "sm1", Transparency::Transparent);
	Modifier->load_graphic(sm10, "sm10", Transparency::Transparent);
    Modifier->load_graphic(sm100, "sm100", Transparency::Transparent);
    Modifier->load_graphic(sm101, "sm101", Transparency::Transparent);
    Modifier->load_graphic(sm102, "sm102", Transparency::Transparent);
    Modifier->load_graphic(sm103, "sm103", Transparency::Transparent);
	Modifier->load_graphic(sm104, "sm104", Transparency::Transparent);
    Modifier->load_graphic(sm105, "sm105", Transparency::Transparent);
    Modifier->load_graphic(sm106, "sm106", Transparency::Transparent);
    Modifier->load_graphic(sm107, "sm107", Transparency::Transparent);
    Modifier->load_graphic(sm108, "sm108", Transparency::Transparent);
	Modifier->load_graphic(sm109, "sm109", Transparency::Transparent);
    Modifier->load_graphic(sm11, "sm11", Transparency::Transparent);
    Modifier->load_graphic(sm110, "sm110", Transparency::Transparent);
    Modifier->load_graphic(sm111, "sm111", Transparency::Transparent);
    Modifier->load_graphic(sm112, "sm112", Transparency::Transparent);
	Modifier->load_graphic(sm115, "sm115", Transparency::Transparent);
    Modifier->load_graphic(sm117, "sm117", Transparency::Transparent);
    Modifier->load_graphic(sm12, "sm12", Transparency::Transparent);
    Modifier->load_graphic(sm129, "sm129", Transparency::Transparent);
    Modifier->load_graphic(sm129striped, "sm129striped", Transparency::Transparent);
	Modifier->load_graphic(sm13, "sm13", Transparency::Transparent);
    Modifier->load_graphic(sm130, "sm130", Transparency::Transparent);
    Modifier->load_graphic(sm130striped, "sm130striped", Transparency::Transparent);
    Modifier->load_graphic(sm131striped, "sm131striped", Transparency::Transparent);
    Modifier->load_graphic(sm132, "sm132", Transparency::Transparent);
	Modifier->load_graphic(sm133, "sm133", Transparency::Transparent);
    Modifier->load_graphic(sm134, "sm134", Transparency::Transparent);
    Modifier->load_graphic(sm135, "sm135", Transparency::Transparent);
    Modifier->load_graphic(sm136, "sm136", Transparency::Transparent);
    Modifier->load_graphic(sm137, "sm137", Transparency::Transparent);
	Modifier->load_graphic(sm138, "sm138", Transparency::Transparent);
    Modifier->load_graphic(sm139, "sm139", Transparency::Transparent);
    Modifier->load_graphic(sm14, "sm14", Transparency::Transparent);
    Modifier->load_graphic(sm15, "sm15", Transparency::Transparent);
    Modifier->load_graphic(sm16, "sm16", Transparency::Transparent);
	Modifier->load_graphic(sm18, "sm18", Transparency::Transparent);
    Modifier->load_graphic(sm19, "sm19", Transparency::Transparent);
    Modifier->load_graphic(sm2, "sm2", Transparency::Transparent);
    Modifier->load_graphic(sm20, "sm20", Transparency::Transparent);
    Modifier->load_graphic(sm21, "sm21", Transparency::Transparent);
	Modifier->load_graphic(sm22, "sm22", Transparency::Transparent);
    Modifier->load_graphic(sm23, "sm23", Transparency::Transparent);
    Modifier->load_graphic(sm24, "sm24", Transparency::Transparent);
    Modifier->load_graphic(sm25, "sm25", Transparency::Transparent);
    Modifier->load_graphic(sm26, "sm26", Transparency::Transparent);
	Modifier->load_graphic(sm27, "sm27", Transparency::Transparent);
    Modifier->load_graphic(sm28, "sm28", Transparency::Transparent);
    Modifier->load_graphic(sm29, "sm29", Transparency::Transparent);
    Modifier->load_graphic(sm3, "sm3", Transparency::Transparent);
    Modifier->load_graphic(sm30, "sm30", Transparency::Transparent);
	Modifier->load_graphic(sm31, "sm31", Transparency::Transparent);
    Modifier->load_graphic(sm32, "sm32", Transparency::Transparent);
    Modifier->load_graphic(sm33, "sm33", Transparency::Transparent);
    Modifier->load_graphic(sm34, "sm34", Transparency::Transparent);
    Modifier->load_graphic(sm35, "sm35", Transparency::Transparent);
	Modifier->load_graphic(sm36, "sm36", Transparency::Transparent);
    Modifier->load_graphic(sm37, "sm37", Transparency::Transparent);
    Modifier->load_graphic(sm38, "sm38", Transparency::Transparent);
    Modifier->load_graphic(sm39, "sm39", Transparency::Transparent);
    Modifier->load_graphic(sm4, "sm4", Transparency::Transparent);
	Modifier->load_graphic(sm40, "sm40", Transparency::Transparent);
    Modifier->load_graphic(sm41, "sm41", Transparency::Transparent);
    Modifier->load_graphic(sm42, "sm42", Transparency::Transparent);
    Modifier->load_graphic(sm43, "sm43", Transparency::Transparent);
    Modifier->load_graphic(sm44, "sm44", Transparency::Transparent);
	Modifier->load_graphic(sm45, "sm45", Transparency::Transparent);
    Modifier->load_graphic(sm46, "sm46", Transparency::Transparent);
    Modifier->load_graphic(sm47, "sm47", Transparency::Transparent);
    Modifier->load_graphic(sm48, "sm48", Transparency::Transparent);
    Modifier->load_graphic(sm49, "sm49", Transparency::Transparent);
	Modifier->load_graphic(sm5, "sm5", Transparency::Transparent);
    Modifier->load_graphic(sm50, "sm50", Transparency::Transparent);
    Modifier->load_graphic(sm51, "sm51", Transparency::Transparent);
    Modifier->load_graphic(sm52, "sm52", Transparency::Transparent);
    Modifier->load_graphic(sm53, "sm53", Transparency::Transparent);
	Modifier->load_graphic(sm54, "sm54", Transparency::Transparent);
    Modifier->load_graphic(sm55, "sm55", Transparency::Transparent);
    Modifier->load_graphic(sm56, "sm56", Transparency::Transparent);
    Modifier->load_graphic(sm57, "sm57", Transparency::Transparent);
    Modifier->load_graphic(sm58, "sm58", Transparency::Transparent);
	Modifier->load_graphic(sm59, "sm59", Transparency::Transparent);
    Modifier->load_graphic(sm6, "sm6", Transparency::Transparent);
    Modifier->load_graphic(sm60, "sm60", Transparency::Transparent);
    Modifier->load_graphic(sm61, "sm61", Transparency::Transparent);
    Modifier->load_graphic(sm62, "sm62", Transparency::Transparent);
	Modifier->load_graphic(sm63, "sm63", Transparency::Transparent);
    Modifier->load_graphic(sm64, "sm64", Transparency::Transparent);
    Modifier->load_graphic(sm65, "sm65", Transparency::Transparent);
    Modifier->load_graphic(sm66, "sm66", Transparency::Transparent);
    Modifier->load_graphic(sm67, "sm67", Transparency::Transparent);
	Modifier->load_graphic(sm7, "sm7", Transparency::Transparent);
    Modifier->load_graphic(sm76, "sm76", Transparency::Transparent);
    Modifier->load_graphic(sm76striped, "sm76striped", Transparency::Transparent);
    Modifier->load_graphic(sm77, "sm77", Transparency::Transparent);
    Modifier->load_graphic(sm77striped, "sm77striped", Transparency::Transparent);
	Modifier->load_graphic(sm78, "sm78", Transparency::Transparent);
    Modifier->load_graphic(sm78striped, "sm78striped", Transparency::Transparent);
    Modifier->load_graphic(sm79, "sm79", Transparency::Transparent);
    Modifier->load_graphic(sm79striped, "sm79striped", Transparency::Transparent);
    Modifier->load_graphic(sm8, "sm8", Transparency::Transparent);
	Modifier->load_graphic(sm80, "sm80", Transparency::Transparent);
    Modifier->load_graphic(sm81, "sm81", Transparency::Transparent);
    Modifier->load_graphic(sm82, "sm82", Transparency::Transparent);
    Modifier->load_graphic(sm83, "sm83", Transparency::Transparent);
    Modifier->load_graphic(sm84, "sm84", Transparency::Transparent);
	Modifier->load_graphic(sm85, "sm85", Transparency::Transparent);
    Modifier->load_graphic(sm86, "sm86", Transparency::Transparent);
    Modifier->load_graphic(sm87, "sm87", Transparency::Transparent);
    Modifier->load_graphic(sm88, "sm88", Transparency::Transparent);
    Modifier->load_graphic(sm89, "sm89", Transparency::Transparent);
	Modifier->load_graphic(sm9, "sm9", Transparency::Transparent);
    Modifier->load_graphic(sm90, "sm90", Transparency::Transparent);
    Modifier->load_graphic(sm91, "sm91", Transparency::Transparent);
    Modifier->load_graphic(sm92, "sm92", Transparency::Transparent);
	Modifier->load_graphic(sm93, "sm93", Transparency::Transparent);
    Modifier->load_graphic(sm94, "sm94", Transparency::Transparent);
    Modifier->load_graphic(sm95, "sm95", Transparency::Transparent);
    Modifier->load_graphic(sm96, "sm96", Transparency::Transparent);
    Modifier->load_graphic(sm96striped, "sm96striped", Transparency::Transparent);
    Modifier->load_graphic(sm97, "sm97", Transparency::Transparent);
    Modifier->load_graphic(sm98, "sm98", Transparency::Transparent);
	Modifier->load_graphic(sm99, "sm99", Transparency::Transparent);
    Modifier->load_graphic(smBlack, "smBlack", Transparency::Transparent);
    Modifier->load_graphic(smBlue, "smBlue", Transparency::Transparent);
    Modifier->load_graphic(smBrightGreen, "smBrightGreen", Transparency::Transparent);
    Modifier->load_graphic(smCaramel, "smCaramel", Transparency::Transparent);
    Modifier->load_graphic(smCyan, "smCyan", Transparency::Transparent);
    Modifier->load_graphic(smLC, "smLC", Transparency::Transparent);
	Modifier->load_graphic(smLightBlue, "smLightBlue", Transparency::Transparent);
    Modifier->load_graphic(smMagenta, "smMagenta", Transparency::Transparent);
    Modifier->load_graphic(smName, "smName", Transparency::Transparent);
    Modifier->load_graphic(smOrange, "smOrange", Transparency::Transparent);
    Modifier->load_graphic(smPaleGreen, "smPaleGreen", Transparency::Transparent);
    Modifier->load_graphic(smRed, "smRed", Transparency::Transparent);
    Modifier->load_graphic(smYellow, "smYellow", Transparency::Transparent);
	Modifier->load_graphic(smTransparent, "smTransparent", Transparency::Transparent);
    Modifier->load_graphic(TempBackground, "TempBackground", Transparency::Transparent);
    Modifier->load_graphic(TempHeadCode, "TempHeadCode", Transparency::Transparent);
    Modifier->load_graphic(UnderHFootbridge, "UnderHFootbridge", Transparency::Transparent);
	Modifier->load_graphic(UnderVFootbridge, "UnderVFootbridge", Transparency::Transparent);


	//failed signal graphics at v2.13.0
	Modifier->load_graphic(FSig68, "FSig68", Transparency::Transparent);
	Modifier->load_graphic(FSig69, "FSig69", Transparency::Transparent);
	Modifier->load_graphic(FSig70, "FSig70", Transparency::Transparent);
	Modifier->load_graphic(FSig71, "FSig71", Transparency::Transparent);
	Modifier->load_graphic(FSig72, "FSig72", Transparency::Transparent);
	Modifier->load_graphic(FSig73, "FSig73", Transparency::Transparent);
	Modifier->load_graphic(FSig74, "FSig74", Transparency::Transparent);
	Modifier->load_graphic(FSig75, "FSig75", Transparency::Transparent);

	Modifier->load_graphic(FGSig68, "FGSig68", Transparency::Transparent);
	Modifier->load_graphic(FGSig69, "FGSig69", Transparency::Transparent);
	Modifier->load_graphic(FGSig70, "FGSig70", Transparency::Transparent);
	Modifier->load_graphic(FGSig71, "FGSig71", Transparency::Transparent);
	Modifier->load_graphic(FGSig72, "FGSig72", Transparency::Transparent);
	Modifier->load_graphic(FGSig73, "FGSig73", Transparency::Transparent);
	Modifier->load_graphic(FGSig74, "FGSig74", Transparency::Transparent);
	Modifier->load_graphic(FGSig75, "FGSig75", Transparency::Transparent);

	// extra from bmSolidBgnd bitmap file but transparent
	Modifier->load_graphic(bmTransparentBgnd, "bmSolidBgnd", Transparency::Transparent);

    // level crossing graphics
	Modifier->load_graphic(LCBothHor, "LCBothHor", Transparency::Transparent);
	Modifier->load_graphic(LCBothHorMan, "LCBothHorMan", Transparency::Transparent);
	Modifier->load_graphic(LCBotHor, "LCBotHor", Transparency::Transparent);
	Modifier->load_graphic(LCBotHorMan, "LCBotHorMan", Transparency::Transparent);
	Modifier->load_graphic(LCBothVer, "LCBothVer", Transparency::Transparent);
	Modifier->load_graphic(LCBothVerMan, "LCBothVerMan", Transparency::Transparent);
	Modifier->load_graphic(LCLHSVer, "LCLHSVer", Transparency::Transparent);
	Modifier->load_graphic(LCLHSVerMan, "LCLHSVerMan", Transparency::Transparent);
	Modifier->load_graphic(LCPlain, "LCPlain", Transparency::Transparent);
	Modifier->load_graphic(LCPlainMan, "LCPlainMan", Transparency::Transparent);
	Modifier->load_graphic(LCRHSVer, "LCRHSVer", Transparency::Transparent);
	Modifier->load_graphic(LCRHSVerMan, "LCRHSVerMan", Transparency::Transparent);
	Modifier->load_graphic(LCTopHor, "LCTopHor", Transparency::Transparent);
	Modifier->load_graphic(LCTopHorMan, "LCTopHorMan", Transparency::Transparent);
;

    HeatMapGraphic->PixelFormat = pf32bit;
    HeatMapGraphic->Height = 16;
    HeatMapGraphic->Width = 16;
    HeatMapGraphic->Transparent = true;
    HeatMapGraphic->TransparentColor = clB5G5R5;


    // additional pointers copied from existing pointers
    sm68 = sm1;
    sm69 = sm1;
    sm70 = sm2;
    sm71 = sm2;
    sm72 = sm19;
    sm73 = sm18;
    sm74 = sm18;
    sm75 = sm19;
    sm113 = sm111;
    sm114 = sm112;
    sm116 = sm115;
    sm118 = sm117;
    sm119 = sm114;
    sm120 = sm117;
    sm121 = sm115;
    sm122 = sm111;
    sm123 = sm111;
    sm124 = sm115;
    sm125 = sm1;
    sm126 = sm1;
    sm127 = sm2;
    sm128 = sm2;

    // non-transparent graphics
	Modifier->load_graphic(Code_a, "Code_a", Transparency::NoTransparency);
    Modifier->load_graphic(Code_b, "Code_b", Transparency::NoTransparency);
    Modifier->load_graphic(Code_c, "Code_c", Transparency::NoTransparency);
    Modifier->load_graphic(Code_d, "Code_d", Transparency::NoTransparency);
    Modifier->load_graphic(Code_e, "Code_e", Transparency::NoTransparency);
    Modifier->load_graphic(Code_f, "Code_f", Transparency::NoTransparency);
    Modifier->load_graphic(Code_g, "Code_g", Transparency::NoTransparency);
    Modifier->load_graphic(Code_h, "Code_h", Transparency::NoTransparency);
    Modifier->load_graphic(Code_i, "Code_i", Transparency::NoTransparency);
    Modifier->load_graphic(Code_j, "Code_j", Transparency::NoTransparency);
    Modifier->load_graphic(Code_k, "Code_k", Transparency::NoTransparency);
    Modifier->load_graphic(Code_l, "Code_l", Transparency::NoTransparency);
    Modifier->load_graphic(Code_m, "Code_m", Transparency::NoTransparency);
    Modifier->load_graphic(Code_n, "Code_n", Transparency::NoTransparency);
    Modifier->load_graphic(Code_o, "Code_o", Transparency::NoTransparency);
    Modifier->load_graphic(Code_p, "Code_p", Transparency::NoTransparency);
    Modifier->load_graphic(Code_q, "Code_q", Transparency::NoTransparency);
    Modifier->load_graphic(Code_r, "Code_r", Transparency::NoTransparency);
    Modifier->load_graphic(Code_s, "Code_s", Transparency::NoTransparency);
    Modifier->load_graphic(Code_t, "Code_t", Transparency::NoTransparency);
    Modifier->load_graphic(Code_u, "Code_u", Transparency::NoTransparency);
    Modifier->load_graphic(Code_v, "Code_v", Transparency::NoTransparency);
    Modifier->load_graphic(Code_w, "Code_w", Transparency::NoTransparency);
    Modifier->load_graphic(Code_x, "Code_x", Transparency::NoTransparency);
    Modifier->load_graphic(Code_y, "Code_y", Transparency::NoTransparency);
    Modifier->load_graphic(Code_z, "Code_z", Transparency::NoTransparency);
    Modifier->load_graphic(Code0, "Code0", Transparency::NoTransparency);
    Modifier->load_graphic(Code1, "Code1", Transparency::NoTransparency);
    Modifier->load_graphic(Code2, "Code2", Transparency::NoTransparency);
    Modifier->load_graphic(Code3, "Code3", Transparency::NoTransparency);
    Modifier->load_graphic(Code4, "Code4", Transparency::NoTransparency);
    Modifier->load_graphic(Code5, "Code5", Transparency::NoTransparency);
    Modifier->load_graphic(Code6, "Code6", Transparency::NoTransparency);
    Modifier->load_graphic(Code7, "Code7", Transparency::NoTransparency);
    Modifier->load_graphic(Code8, "Code8", Transparency::NoTransparency);
    Modifier->load_graphic(Code9, "Code9", Transparency::NoTransparency);
    Modifier->load_graphic(CodeA, "CodeA", Transparency::NoTransparency);
    Modifier->load_graphic(CodeB, "CodeB", Transparency::NoTransparency);
    Modifier->load_graphic(CodeC, "CodeC", Transparency::NoTransparency);
    Modifier->load_graphic(CodeD, "CodeD", Transparency::NoTransparency);
    Modifier->load_graphic(CodeE, "CodeE", Transparency::NoTransparency);
    Modifier->load_graphic(CodeF, "CodeF", Transparency::NoTransparency);
    Modifier->load_graphic(CodeG, "CodeG", Transparency::NoTransparency);
    Modifier->load_graphic(CodeH, "CodeH", Transparency::NoTransparency);
    Modifier->load_graphic(CodeI, "CodeI", Transparency::NoTransparency);
    Modifier->load_graphic(CodeJ, "CodeJ", Transparency::NoTransparency);
    Modifier->load_graphic(CodeK, "CodeK", Transparency::NoTransparency);
    Modifier->load_graphic(CodeL, "CodeL", Transparency::NoTransparency);
    Modifier->load_graphic(CodeM, "CodeM", Transparency::NoTransparency);
    Modifier->load_graphic(CodeN, "CodeN", Transparency::NoTransparency);
    Modifier->load_graphic(CodeO, "CodeO", Transparency::NoTransparency);
    Modifier->load_graphic(CodeP, "CodeP", Transparency::NoTransparency);
    Modifier->load_graphic(CodeQ, "CodeQ", Transparency::NoTransparency);
    Modifier->load_graphic(CodeR, "CodeR", Transparency::NoTransparency);
    Modifier->load_graphic(CodeS, "CodeS", Transparency::NoTransparency);
    Modifier->load_graphic(CodeT, "CodeT", Transparency::NoTransparency);
    Modifier->load_graphic(CodeU, "CodeU", Transparency::NoTransparency);
    Modifier->load_graphic(CodeV, "CodeV", Transparency::NoTransparency);
    Modifier->load_graphic(CodeW, "CodeW", Transparency::NoTransparency);
    Modifier->load_graphic(CodeX, "CodeX", Transparency::NoTransparency);
    Modifier->load_graphic(CodeY, "CodeY", Transparency::NoTransparency);
    Modifier->load_graphic(CodeZ, "CodeZ", Transparency::NoTransparency);
    Modifier->load_graphic(bmSolidBgnd, "bmSolidBgnd", Transparency::NoTransparency);
    Modifier->load_graphic(smSolidBgnd, "smSolidBgnd", Transparency::NoTransparency);
    Modifier->load_graphic(bmDiagonalSignalBlank, "bmDiagonalSignalBlank", Transparency::NoTransparency);
    Modifier->load_graphic(bmPointBlank, "bmPointBlank", Transparency::NoTransparency);
    Modifier->load_graphic(bmStraightEWSignalBlank, "bmStraightEWSignalBlank", Transparency::NoTransparency);
    Modifier->load_graphic(bmStraightNSSignalBlank, "bmStraightNSSignalBlank", Transparency::NoTransparency);


}

// ---------------------------------------------------------------------------


void TRailGraphics::ChangeTransparentColour(Graphics::TBitmap *BitmapIn, Graphics::TBitmap *BitmapOut, TColor OldTransparentColour)
{
	const TColor NewTransparentColour{Utilities->clTransparent};

	// Change OldTransparentColour to NewTransparentColour, if NewTransparentColour not white change black to white
	// else change white to black
	if(NewTransparentColour == OldTransparentColour)
	{
		return; // already stored

	}
	Utilities->CallLog.push_back(Utilities->TimeStamp() + ",ChangeTransparentColour," + AnsiString(NewTransparentColour) + "," +
								 AnsiString(OldTransparentColour));
	BitmapOut->Height = BitmapIn->Height;
	BitmapOut->Width = BitmapIn->Width;
	for(int x = 0; x < BitmapIn->Width; x++)
	{
		for(int y = 0; y < BitmapIn->Height; y++)
		{
			if(NewTransparentColour != clB5G5R5) // new is dark, old could be dark or light
			{
				if(BitmapIn->Canvas->Pixels[x][y] == OldTransparentColour)
				{
					BitmapOut->Canvas->Pixels[x][y] = NewTransparentColour;
				}
				else if(BitmapIn->Canvas->Pixels[x][y] == clB0G0R0)
				// black - if OldTransparentColour was black (or any dark colour) then track will be white already
				{
					BitmapOut->Canvas->Pixels[x][y] = clB5G5R5; // white
				}
				else
				{
					BitmapOut->Canvas->Pixels[x][y] = BitmapIn->Canvas->Pixels[x][y];
				}
			}
			else
			{
				// new is white, old must be dark else new = old & wouldn't reach here
				if(BitmapIn->Canvas->Pixels[x][y] == OldTransparentColour)
				{
					BitmapOut->Canvas->Pixels[x][y] = NewTransparentColour;
				}
				else if(BitmapIn->Canvas->Pixels[x][y] == clB5G5R5) // white
				{
					BitmapOut->Canvas->Pixels[x][y] = clB0G0R0; // black
				}
				else
				{
					BitmapOut->Canvas->Pixels[x][y] = BitmapIn->Canvas->Pixels[x][y];
				}
			}
		}
	}
	BitmapOut->TransparentColor = NewTransparentColour;
	Utilities->CallLogPop(1795);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ChangeForegroundColour(int Caller, Graphics::TBitmap *BitmapIn, Graphics::TBitmap *BitmapOut, TColor NewForegroundColour,
										   TColor BackgroundColour)
{
	Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",ChangeForegroundColour," + AnsiString(NewForegroundColour) + "," +
								 AnsiString(BackgroundColour));
    BitmapOut->Height = BitmapIn->Height;
    BitmapOut->Width = BitmapIn->Width;
    for(int x = 0; x < BitmapIn->Width; x++)
    {
        for(int y = 0; y < BitmapIn->Height; y++)
        {
            if(BitmapIn->Canvas->Pixels[x][y] != BackgroundColour)
            {
                BitmapOut->Canvas->Pixels[x][y] = NewForegroundColour;
            }
            else
            {
                BitmapOut->Canvas->Pixels[x][y] = BackgroundColour;
            }
        }
    }
    Utilities->CallLogPop(1480);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ChangeForegroundColour2(int Caller, Graphics::TBitmap *BitmapIn, Graphics::TBitmap *BitmapOut, TColor NewForegroundColour,
                                            TColor BackgroundColour)
{
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",ChangeForegroundColour2," + AnsiString(NewForegroundColour) + "," +
                                 AnsiString(BackgroundColour));
    Graphics::TBitmap *TempBitmapOut = new Graphics::TBitmap;

    TempBitmapOut->Assign(BitmapIn); // in case BitmapOut isn't fully defined at this stage
    TRect FullRect(0, 0, BitmapIn->Width, BitmapIn->Height); // the full size of both bitmaps

    TempBitmapOut->Canvas->Brush->Color = BackgroundColour; // BrushStyle default is solid so leave at that
    TempBitmapOut->Canvas->FillRect(FullRect);
    for(int x = 0; x < BitmapIn->Width; x++)
    {
        for(int y = 0; y < BitmapIn->Height; y++)
        {
            if(BitmapIn->Canvas->Pixels[x][y] != BackgroundColour)
            {
                TempBitmapOut->Canvas->Pixels[x][y] = NewForegroundColour;
            }
        }
    }
    BitmapOut->Assign(TempBitmapOut);
    delete TempBitmapOut;
    Utilities->CallLogPop(2101);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ChangeSpecificColour(int Caller, Graphics::TBitmap *BitmapIn, Graphics::TBitmap *BitmapOut, TColor ColourToBeChanged, TColor NewColour)
{
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",ChangeSpecificColour," + AnsiString(ColourToBeChanged) + "," +
                                 AnsiString(NewColour));
    BitmapOut->Height = BitmapIn->Height;
    BitmapOut->Width = BitmapIn->Width;
    for(int x = 0; x < BitmapIn->Width; x++)
    {
        for(int y = 0; y < BitmapIn->Height; y++)
        {
            if(BitmapIn->Canvas->Pixels[x][y] == ColourToBeChanged)
            {
                BitmapOut->Canvas->Pixels[x][y] = NewColour;
            }
        }
    }
    Utilities->CallLogPop(1481);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ChangeBackgroundColour(int Caller, Graphics::TBitmap *BitmapIn, Graphics::TBitmap* BitmapOut, TColor NewBackgroundColour,
                                           TColor OldBackgroundColour, bool &ColourError)
{
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",ChangeBackgroundColour," + AnsiString(NewBackgroundColour) + "," +
                                 AnsiString(OldBackgroundColour));
    ColourError = false;
    BitmapOut->Height = BitmapIn->Height;
    BitmapOut->Width = BitmapIn->Width;
    bool OneChanged = false;

    for(int x = 0; x < BitmapIn->Width; x++)
    {
        for(int y = 0; y < BitmapIn->Height; y++)
        {
            if(BitmapIn->Canvas->Pixels[x][y] == OldBackgroundColour)
            {
                BitmapOut->Canvas->Pixels[x][y] = NewBackgroundColour;
                if(!OneChanged)
                {
                    if(BitmapOut->Canvas->Pixels[x][y] != NewBackgroundColour)
                    {
                        // can be different if the palette of the pixel is different from that expected
                        ColourError = true;
// throw Exception("BackgroundColour change incorrect - actual = " + AnsiString((int)BitmapOut->Canvas->Pixels[x][y]) + ", expected " + AnsiString((int)NewBackgroundColour));
                    }
                }
                OneChanged = true;
            }
            else
            {
                BitmapOut->Canvas->Pixels[x][y] = BitmapIn->Canvas->Pixels[x][y];
            }
        }
    }
    Utilities->CallLogPop(1482);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ChangeBackgroundColour2(int Caller, Graphics::TBitmap *BitmapIn, Graphics::TBitmap* BitmapOut, TColor NewBackgroundColour,
                                            TColor OldBackgroundColour)
{
// superseded by ChangeBackgroundColour3 using direct pixel manipulation
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",ChangeBackgroundColour2," + AnsiString(NewBackgroundColour) + "," +
                                 AnsiString(OldBackgroundColour));
    Graphics::TBitmap *TempBitmapOut = new Graphics::TBitmap;

    TempBitmapOut->Assign(BitmapIn); // in case BitmapOut isn't fully defined at this stage
    TRect FullRect(0, 0, BitmapIn->Width, BitmapIn->Height); // the full size of both bitmaps

    TempBitmapOut->Canvas->Brush->Color = NewBackgroundColour; // BrushStyle default is solid so leave at that
    TempBitmapOut->Canvas->FillRect(FullRect); // now all coloured same as new background

// TempBitmapOut->Canvas->CopyMode = cmSrcAnd; //these lines instead of the for.. next pixel loop didn't work, they gave distorted
// TempBitmapOut->Canvas->CopyRect(FullRect, BitmapIn->Canvas, FullRect); //colours, but worked OK without the Assign for a reason I couldn't fathom

    for(int x = 0; x < BitmapIn->Width; x++)
    {
        for(int y = 0; y < BitmapIn->Height; y++)
        {
            {
                if(BitmapIn->Canvas->Pixels[x][y] != OldBackgroundColour)
                {
                    TempBitmapOut->Canvas->Pixels[x][y] = BitmapIn->Canvas->Pixels[x][y];
                }
            }
        }
    }

    BitmapOut->Assign(TempBitmapOut);
    delete TempBitmapOut;
    Utilities->CallLogPop(2102);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ChangeBackgroundColour3(int Caller, Graphics::TBitmap *BitmapIn, Graphics::TBitmap* BitmapOut, TColor NewBackgroundColour,
                                            TColor OldBackgroundColour)
{  //NB  if use Byte* then have to use pf8bitbit form graphics, for pf24bit need TRGBTriple* - see Main Help for Scanline (C++)
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",ChangeBackgroundColour3" + AnsiString(NewBackgroundColour) + "," +
                                 AnsiString(OldBackgroundColour));
    Byte *SLPtrIn; // pointer to the ScanLine values in BitmapIn
    Byte *SLPtrOut; // pointer to the ScanLine values in TempBitmapOut
    Graphics::TBitmap *TempBitmapOut = new Graphics::TBitmap;

    TempBitmapOut->Assign(BitmapIn); // in case BitmapOut isn't fully defined at this stage
    int NewBGColourNumber = ColNametoNumber(0, NewBackgroundColour);
    int OldBGColourNumber = ColNametoNumber(1, OldBackgroundColour);

    for(int x = 0; x < BitmapIn->Height; x++)
    {
        SLPtrIn = reinterpret_cast<Byte*>(BitmapIn->ScanLine[x]);
        SLPtrOut = reinterpret_cast<Byte*>(TempBitmapOut->ScanLine[x]);
        for(int y = 0; y < BitmapIn->Width; y++)
        {
            if(SLPtrIn[y] == OldBGColourNumber)
            {
                SLPtrOut[y] = NewBGColourNumber;
            }
            else
            {
                SLPtrOut[y] = SLPtrIn[y];
            }
        }
    }
    BitmapOut->Assign(TempBitmapOut);
    delete TempBitmapOut;
    Utilities->CallLogPop(2103);
}

// ---------------------------------------------------------------------------

int TRailGraphics::ColNametoNumber(int Caller, TColor Colour)
{
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",ColNametoNumber" + AnsiString(Colour));
    int Number;

    if(Colour == clB0G0R0) // black       //used these in place of case statements because 64 bit compiler issued warnings
    {                                     //that not using the enum names, and several fell inbetween the enum names
        Number = 0x0;
    }
    else if(Colour == clB5G5R5) // white
    {
        Number = 0xd7;
    }
    else if(Colour == clBufferAttentionNeeded)
    {
        Number = 0x23;
    }
    else if(Colour == clBufferStopBackground)
    {
        Number = 0xb3;
    }
    else if(Colour == clCallOnBackground)
    {
        Number = 0xbf;
    }
    else if(Colour == clCrashedBackground) // covers DerailedBackground because that has the same colour number
    {
        Number = 0xb4;
    }
    else if(Colour == clNormalBackground)
    {
        Number = 0xac;
    }
    else if(Colour == clSignallerStopped)
    {
        Number = 0xcf;
    }
    else if(Colour == clSignalStopBackground)
    {
        Number = 0x66;
    }
    else if(Colour == clSPADBackground)
    {
        Number = 0xd2;
    }
    else if(Colour == clStationStopBackground)
    {
        Number = 0xb2;
    }
    else if(Colour == clStoppedTrainInFront)
    {
        Number = 0x83;
    }
    else if(Colour == clTRSBackground)
    {
        Number = 0xd1;
    }
    else if(Colour == clTrainFailedBackground) // added at v2.4.0
    {
        Number = 0xc0;
    }
    else
    {
        Number = 0xac; // normal background
    }


/*
    switch(Colour)
    {
        case clB0G0R0: // black
            Number = 0x0;
            break;

        case clB5G5R5: // white
            Number = 0xd7;
            break;

        case clBufferAttentionNeeded:
            Number = 0x23;
            break;

        case clBufferStopBackground:
            Number = 0xb3;
            break;

        case clCallOnBackground:
            Number = 0xbf;
            break;

        case clCrashedBackground: // covers DerailedBackground because that has the same colour number
            Number = 0xb4;
            break;

        case clNormalBackground:
            Number = 0xac;
            break;

        case clSignallerStopped:
            Number = 0xcf;
            break;

        case clSignalStopBackground:
            Number = 0x66;
            break;

        case clSPADBackground:
            Number = 0xd2;
            break;

        case clStationStopBackground:
            Number = 0xb2;
            break;

        case clStoppedTrainInFront:
            Number = 0x83;
            break;

        case clTRSBackground:
            Number = 0xd1;
            break;

        case clTrainFailedBackground: // added at v2.4.0
            Number = 0xc0;
            break;

        default:
    // UnicodeString MessageStr = "Can't find required colour - normal background colour will be used.");
    // Application->MessageBox(MessageStr.c_str(), L"", MB_OK);  don't give message as can be called when operating (need StopTTClockMessage)
            Number = 0xac; // normal background
            break;
*/
    Utilities->CallLogPop(2104);
    return(Number);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ChangeAllTransparentColours()
{
	const TColor old_transparent_color_ = Cache.previous_color;

    if(Utilities->clTransparent == old_transparent_color_)
    {
        return; // already stored

    }
	ChangeTransparentColour(BlackCircle, BlackCircle, old_transparent_color_); //added at v2.13.0
	ChangeTransparentColour(bm10, bm10, old_transparent_color_);
	ChangeTransparentColour(bm100, bm100, old_transparent_color_);
	ChangeTransparentColour(bm101, bm101, old_transparent_color_);
	ChangeTransparentColour(bm106, bm106, old_transparent_color_);
	ChangeTransparentColour(bm10Diverging, bm10Diverging, old_transparent_color_);
	ChangeTransparentColour(bm10Straight, bm10Straight, old_transparent_color_);
	ChangeTransparentColour(bm11, bm11, old_transparent_color_);
	ChangeTransparentColour(bm11Diverging, bm11Diverging, old_transparent_color_);
	ChangeTransparentColour(bm11Straight, bm11Straight, old_transparent_color_);
    ChangeTransparentColour(bm12, bm12, old_transparent_color_);
	ChangeTransparentColour(bm12Diverging, bm12Diverging, old_transparent_color_);
	ChangeTransparentColour(bm12Straight, bm12Straight, old_transparent_color_);
	ChangeTransparentColour(bm13, bm13, old_transparent_color_);
	ChangeTransparentColour(bm132, bm132, old_transparent_color_);
	ChangeTransparentColour(bm132LeftFork, bm132LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm132RightFork, bm132RightFork, old_transparent_color_);
	ChangeTransparentColour(bm133, bm133, old_transparent_color_);
	ChangeTransparentColour(bm133LeftFork, bm133LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm133RightFork, bm133RightFork, old_transparent_color_);
	ChangeTransparentColour(bm134, bm134, old_transparent_color_);
	ChangeTransparentColour(bm134LeftFork, bm134LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm134RightFork, bm134RightFork, old_transparent_color_);
	ChangeTransparentColour(bm135, bm135, old_transparent_color_);
	ChangeTransparentColour(bm135LeftFork, bm135LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm135RightFork, bm135RightFork, old_transparent_color_);
	ChangeTransparentColour(bm136, bm136, old_transparent_color_);
	ChangeTransparentColour(bm136LeftFork, bm136LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm136RightFork, bm136RightFork, old_transparent_color_);
	ChangeTransparentColour(bm137, bm137, old_transparent_color_);
	ChangeTransparentColour(bm137LeftFork, bm137LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm137RightFork, bm137RightFork, old_transparent_color_);
	ChangeTransparentColour(bm138, bm138, old_transparent_color_);
	ChangeTransparentColour(bm138LeftFork, bm138LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm138RightFork, bm138RightFork, old_transparent_color_);
	ChangeTransparentColour(bm139, bm139, old_transparent_color_);
	ChangeTransparentColour(bm139LeftFork, bm139LeftFork, old_transparent_color_);
	ChangeTransparentColour(bm139RightFork, bm139RightFork, old_transparent_color_);
	ChangeTransparentColour(bm13Diverging, bm13Diverging, old_transparent_color_);
	ChangeTransparentColour(bm13Straight, bm13Straight, old_transparent_color_);
	ChangeTransparentColour(bm14, bm14, old_transparent_color_);
	ChangeTransparentColour(bm140, bm140, old_transparent_color_);
	ChangeTransparentColour(bm141, bm141, old_transparent_color_);
	ChangeTransparentColour(bm14Diverging, bm14Diverging, old_transparent_color_);
	ChangeTransparentColour(bm14Straight, bm14Straight, old_transparent_color_);
	ChangeTransparentColour(bm16, bm16, old_transparent_color_);
	ChangeTransparentColour(bm18, bm18, old_transparent_color_);
	ChangeTransparentColour(bm20, bm20, old_transparent_color_);
	ChangeTransparentColour(bm27, bm27, old_transparent_color_);
	ChangeTransparentColour(bm28, bm28, old_transparent_color_);
	ChangeTransparentColour(bm28Diverging, bm28Diverging, old_transparent_color_);
	ChangeTransparentColour(bm28Straight, bm28Straight, old_transparent_color_);
	ChangeTransparentColour(bm29, bm29, old_transparent_color_);
	ChangeTransparentColour(bm29Diverging, bm29Diverging, old_transparent_color_);
	ChangeTransparentColour(bm29Straight, bm29Straight, old_transparent_color_);
	ChangeTransparentColour(bm30, bm30, old_transparent_color_);
	ChangeTransparentColour(bm30Diverging, bm30Diverging, old_transparent_color_);
	ChangeTransparentColour(bm30Straight, bm30Straight, old_transparent_color_);
	ChangeTransparentColour(bm31, bm31, old_transparent_color_);
	ChangeTransparentColour(bm31Diverging, bm31Diverging, old_transparent_color_);
	ChangeTransparentColour(bm31Straight, bm31Straight, old_transparent_color_);
	ChangeTransparentColour(bm32, bm32, old_transparent_color_);
	ChangeTransparentColour(bm32Diverging, bm32Diverging, old_transparent_color_);
	ChangeTransparentColour(bm32Straight, bm32Straight, old_transparent_color_);
	ChangeTransparentColour(bm33, bm33, old_transparent_color_);
	ChangeTransparentColour(bm33Diverging, bm33Diverging, old_transparent_color_);
	ChangeTransparentColour(bm33Straight, bm33Straight, old_transparent_color_);
	ChangeTransparentColour(bm34, bm34, old_transparent_color_);
	ChangeTransparentColour(bm34Diverging, bm34Diverging, old_transparent_color_);
	ChangeTransparentColour(bm34Straight, bm34Straight, old_transparent_color_);
	ChangeTransparentColour(bm35, bm35, old_transparent_color_);
    ChangeTransparentColour(bm35Diverging, bm35Diverging, old_transparent_color_);
    ChangeTransparentColour(bm35Straight, bm35Straight, old_transparent_color_);
	ChangeTransparentColour(bm36, bm36, old_transparent_color_);
    ChangeTransparentColour(bm36Diverging, bm36Diverging, old_transparent_color_);
    ChangeTransparentColour(bm36Straight, bm36Straight, old_transparent_color_);
    ChangeTransparentColour(bm37, bm37, old_transparent_color_);
    ChangeTransparentColour(bm37Diverging, bm37Diverging, old_transparent_color_);
	ChangeTransparentColour(bm37Straight, bm37Straight, old_transparent_color_);
    ChangeTransparentColour(bm38, bm38, old_transparent_color_);
    ChangeTransparentColour(bm38Diverging, bm38Diverging, old_transparent_color_);
    ChangeTransparentColour(bm38Straight, bm38Straight, old_transparent_color_);
    ChangeTransparentColour(bm39, bm39, old_transparent_color_);
	ChangeTransparentColour(bm39Diverging, bm39Diverging, old_transparent_color_);
    ChangeTransparentColour(bm39Straight, bm39Straight, old_transparent_color_);
    ChangeTransparentColour(bm40, bm40, old_transparent_color_);
    ChangeTransparentColour(bm40Diverging, bm40Diverging, old_transparent_color_);
    ChangeTransparentColour(bm40Straight, bm40Straight, old_transparent_color_);
	ChangeTransparentColour(bm41, bm41, old_transparent_color_);
    ChangeTransparentColour(bm41Diverging, bm41Diverging, old_transparent_color_);
    ChangeTransparentColour(bm41Straight, bm41Straight, old_transparent_color_);
    ChangeTransparentColour(bm42, bm42, old_transparent_color_);
    ChangeTransparentColour(bm42Diverging, bm42Diverging, old_transparent_color_);
	ChangeTransparentColour(bm42Straight, bm42Straight, old_transparent_color_);
    ChangeTransparentColour(bm43, bm43, old_transparent_color_);
    ChangeTransparentColour(bm43Diverging, bm43Diverging, old_transparent_color_);
    ChangeTransparentColour(bm43Straight, bm43Straight, old_transparent_color_);
    ChangeTransparentColour(bm45, bm45, old_transparent_color_);
	ChangeTransparentColour(bm46, bm46, old_transparent_color_);
    ChangeTransparentColour(bm50, bm50, old_transparent_color_);
    ChangeTransparentColour(bm51, bm51, old_transparent_color_);
    ChangeTransparentColour(bm53, bm53, old_transparent_color_);
    ChangeTransparentColour(bm54, bm54, old_transparent_color_);
	ChangeTransparentColour(bm56, bm56, old_transparent_color_);
    ChangeTransparentColour(bm59, bm59, old_transparent_color_);
    ChangeTransparentColour(bm65, bm65, old_transparent_color_);
    ChangeTransparentColour(bm68CallingOn, bm68CallingOn, old_transparent_color_);
    ChangeTransparentColour(bm68dblyellow, bm68dblyellow, old_transparent_color_);
	ChangeTransparentColour(bm68grounddblred, bm68grounddblred, old_transparent_color_);
    ChangeTransparentColour(bm68grounddblwhite, bm68grounddblwhite, old_transparent_color_);
    ChangeTransparentColour(bm68green, bm68green, old_transparent_color_);
    ChangeTransparentColour(bm68yellow, bm68yellow, old_transparent_color_);
    ChangeTransparentColour(bm69CallingOn, bm69CallingOn, old_transparent_color_);
	ChangeTransparentColour(bm69dblyellow, bm69dblyellow, old_transparent_color_);
    ChangeTransparentColour(bm69grounddblred, bm69grounddblred, old_transparent_color_);
    ChangeTransparentColour(bm69grounddblwhite, bm69grounddblwhite, old_transparent_color_);
    ChangeTransparentColour(bm69green, bm69green, old_transparent_color_);
    ChangeTransparentColour(bm69yellow, bm69yellow, old_transparent_color_);
	ChangeTransparentColour(bm7, bm7, old_transparent_color_);
    ChangeTransparentColour(bm70CallingOn, bm70CallingOn, old_transparent_color_);
    ChangeTransparentColour(bm70dblyellow, bm70dblyellow, old_transparent_color_);
    ChangeTransparentColour(bm70grounddblred, bm70grounddblred, old_transparent_color_);
    ChangeTransparentColour(bm70grounddblwhite, bm70grounddblwhite, old_transparent_color_);
	ChangeTransparentColour(bm70green, bm70green, old_transparent_color_);
    ChangeTransparentColour(bm70yellow, bm70yellow, old_transparent_color_);
    ChangeTransparentColour(bm71CallingOn, bm71CallingOn, old_transparent_color_);
    ChangeTransparentColour(bm71dblyellow, bm71dblyellow, old_transparent_color_);
    ChangeTransparentColour(bm71grounddblred, bm71grounddblred, old_transparent_color_);
	ChangeTransparentColour(bm71grounddblwhite, bm71grounddblwhite, old_transparent_color_);
    ChangeTransparentColour(bm71green, bm71green, old_transparent_color_);
    ChangeTransparentColour(bm71yellow, bm71yellow, old_transparent_color_);
    ChangeTransparentColour(bm72CallingOn, bm72CallingOn, old_transparent_color_);
    ChangeTransparentColour(bm72dblyellow, bm72dblyellow, old_transparent_color_);
	ChangeTransparentColour(bm72grounddblred, bm72grounddblred, old_transparent_color_);
    ChangeTransparentColour(bm72grounddblwhite, bm72grounddblwhite, old_transparent_color_);
    ChangeTransparentColour(bm72green, bm72green, old_transparent_color_);
    ChangeTransparentColour(bm72yellow, bm72yellow, old_transparent_color_);
    ChangeTransparentColour(bm73, bm73, old_transparent_color_);
	ChangeTransparentColour(bm73CallingOn, bm73CallingOn, old_transparent_color_);
    ChangeTransparentColour(bm73dblyellow, bm73dblyellow, old_transparent_color_);
    ChangeTransparentColour(bm73grounddblred, bm73grounddblred, old_transparent_color_);
    ChangeTransparentColour(bm73grounddblwhite, bm73grounddblwhite, old_transparent_color_);
    ChangeTransparentColour(bm73green, bm73green, old_transparent_color_);
	ChangeTransparentColour(bm73yellow, bm73yellow, old_transparent_color_);
    ChangeTransparentColour(bm74, bm74, old_transparent_color_);
    ChangeTransparentColour(bm74CallingOn, bm74CallingOn, old_transparent_color_);
    ChangeTransparentColour(bm74dblyellow, bm74dblyellow, old_transparent_color_);
    ChangeTransparentColour(bm74grounddblred, bm74grounddblred, old_transparent_color_);
	ChangeTransparentColour(bm74grounddblwhite, bm74grounddblwhite, old_transparent_color_);
    ChangeTransparentColour(bm74green, bm74green, old_transparent_color_);
    ChangeTransparentColour(bm74yellow, bm74yellow, old_transparent_color_);
    ChangeTransparentColour(bm75CallingOn, bm75CallingOn, old_transparent_color_);
    ChangeTransparentColour(bm75dblyellow, bm75dblyellow, old_transparent_color_);
	ChangeTransparentColour(bm75grounddblred, bm75grounddblred, old_transparent_color_);
    ChangeTransparentColour(bm75grounddblwhite, bm75grounddblwhite, old_transparent_color_);
    ChangeTransparentColour(bm75green, bm75green, old_transparent_color_);
    ChangeTransparentColour(bm75yellow, bm75yellow, old_transparent_color_);
    ChangeTransparentColour(bm77, bm77, old_transparent_color_);
	ChangeTransparentColour(bm77Striped, bm77Striped, old_transparent_color_);
    ChangeTransparentColour(bm78, bm78, old_transparent_color_);
    ChangeTransparentColour(bm78Striped, bm78Striped, old_transparent_color_);
    ChangeTransparentColour(bm7Diverging, bm7Diverging, old_transparent_color_);
    ChangeTransparentColour(bm7Straight, bm7Straight, old_transparent_color_);
	ChangeTransparentColour(bm8, bm8, old_transparent_color_);
    ChangeTransparentColour(bm85, bm85, old_transparent_color_);
    ChangeTransparentColour(bm8Diverging, bm8Diverging, old_transparent_color_);
    ChangeTransparentColour(bm8Straight, bm8Straight, old_transparent_color_);
    ChangeTransparentColour(bm9, bm9, old_transparent_color_);
	ChangeTransparentColour(bm93set, bm93set, old_transparent_color_);
    ChangeTransparentColour(bm93unset, bm93unset, old_transparent_color_);
    ChangeTransparentColour(bm94set, bm94set, old_transparent_color_);
    ChangeTransparentColour(bm94unset, bm94unset, old_transparent_color_);
    ChangeTransparentColour(bm9Diverging, bm9Diverging, old_transparent_color_);
	ChangeTransparentColour(bm9Straight, bm9Straight, old_transparent_color_);
    ChangeTransparentColour(bmGreenEllipse, bmGreenEllipse, old_transparent_color_);
    ChangeTransparentColour(bmGreenRect, bmGreenRect, old_transparent_color_);
    ChangeTransparentColour(bmGrid, bmGrid, old_transparent_color_);
    ChangeTransparentColour(bmLightBlueRect, bmLightBlueRect, old_transparent_color_);
	ChangeTransparentColour(bmName, bmName, old_transparent_color_);
    ChangeTransparentColour(bmNameStriped, bmNameStriped, old_transparent_color_);
    ChangeTransparentColour(bmRedEllipse, bmRedEllipse, old_transparent_color_);
    ChangeTransparentColour(bmRedRect, bmRedRect, old_transparent_color_);
    ChangeTransparentColour(bmRtCancELink1, bmRtCancELink1, old_transparent_color_);
	ChangeTransparentColour(bmRtCancELink2, bmRtCancELink2, old_transparent_color_);
    ChangeTransparentColour(bmRtCancELink3, bmRtCancELink3, old_transparent_color_);
    ChangeTransparentColour(bmRtCancELink4, bmRtCancELink4, old_transparent_color_);
    ChangeTransparentColour(bmRtCancELink6, bmRtCancELink6, old_transparent_color_);
    ChangeTransparentColour(bmRtCancELink7, bmRtCancELink7, old_transparent_color_);
	ChangeTransparentColour(bmRtCancELink8, bmRtCancELink8, old_transparent_color_);
    ChangeTransparentColour(bmRtCancELink9, bmRtCancELink9, old_transparent_color_);
    ChangeTransparentColour(br1, br1, old_transparent_color_);
    ChangeTransparentColour(br10, br10, old_transparent_color_);
    ChangeTransparentColour(br11, br11, old_transparent_color_);
	ChangeTransparentColour(br12, br12, old_transparent_color_);
    ChangeTransparentColour(br2, br2, old_transparent_color_);
    ChangeTransparentColour(br3, br3, old_transparent_color_);
    ChangeTransparentColour(br4, br4, old_transparent_color_);
    ChangeTransparentColour(br5, br5, old_transparent_color_);
	ChangeTransparentColour(br6, br6, old_transparent_color_);
    ChangeTransparentColour(br7, br7, old_transparent_color_);
    ChangeTransparentColour(br8, br8, old_transparent_color_);
    ChangeTransparentColour(br9, br9, old_transparent_color_);
    ChangeTransparentColour(Concourse, Concourse, old_transparent_color_);
	ChangeTransparentColour(ConcourseGlyph, ConcourseGlyph, old_transparent_color_);
    ChangeTransparentColour(ConcourseStriped, ConcourseStriped, old_transparent_color_);

    ChangeTransparentColour(CouplingExit1, CouplingExit1, old_transparent_color_); //Multiplayer coupled exit graphics
    ChangeTransparentColour(CouplingExit2, CouplingExit2, old_transparent_color_);
	ChangeTransparentColour(CouplingExit3, CouplingExit3, old_transparent_color_);
    ChangeTransparentColour(CouplingExit4, CouplingExit4, old_transparent_color_);
    ChangeTransparentColour(CouplingExit6, CouplingExit6, old_transparent_color_);
    ChangeTransparentColour(CouplingExit7, CouplingExit7, old_transparent_color_);
    ChangeTransparentColour(CouplingExit8, CouplingExit8, old_transparent_color_);
	ChangeTransparentColour(CouplingExit9, CouplingExit9, old_transparent_color_);

    ChangeTransparentColour(ELk1, ELk1, old_transparent_color_);
    ChangeTransparentColour(ELk2, ELk2, old_transparent_color_);
    ChangeTransparentColour(ELk3, ELk3, old_transparent_color_);
	ChangeTransparentColour(ELk4, ELk4, old_transparent_color_);
    ChangeTransparentColour(ELk6, ELk6, old_transparent_color_);
    ChangeTransparentColour(ELk7, ELk7, old_transparent_color_);
    ChangeTransparentColour(ELk8, ELk8, old_transparent_color_);
    ChangeTransparentColour(ELk9, ELk9, old_transparent_color_);
	ChangeTransparentColour(BlackOctagon, BlackOctagon, old_transparent_color_); //added at v2.13.0
    ChangeTransparentColour(gl1, gl1, old_transparent_color_);
    ChangeTransparentColour(gl10, gl10, old_transparent_color_);
    ChangeTransparentColour(gl100, gl100, old_transparent_color_);
    ChangeTransparentColour(gl101, gl101, old_transparent_color_);
	ChangeTransparentColour(gl102, gl102, old_transparent_color_);
    ChangeTransparentColour(gl103, gl103, old_transparent_color_);
    ChangeTransparentColour(gl104, gl104, old_transparent_color_);
    ChangeTransparentColour(gl105, gl105, old_transparent_color_);
	ChangeTransparentColour(gl106, gl106, old_transparent_color_);
    ChangeTransparentColour(gl107, gl107, old_transparent_color_);
    ChangeTransparentColour(gl108, gl108, old_transparent_color_);
    ChangeTransparentColour(gl109, gl109, old_transparent_color_);
	ChangeTransparentColour(gl11, gl11, old_transparent_color_);
    ChangeTransparentColour(gl110, gl110, old_transparent_color_);
    ChangeTransparentColour(gl111, gl111, old_transparent_color_);
    ChangeTransparentColour(gl112, gl112, old_transparent_color_);
	ChangeTransparentColour(gl113, gl113, old_transparent_color_);
    ChangeTransparentColour(gl114, gl114, old_transparent_color_);
    ChangeTransparentColour(gl115, gl115, old_transparent_color_);
    ChangeTransparentColour(gl116, gl116, old_transparent_color_);
	ChangeTransparentColour(gl117, gl117, old_transparent_color_);
    ChangeTransparentColour(gl118, gl118, old_transparent_color_);
    ChangeTransparentColour(gl119, gl119, old_transparent_color_);
    ChangeTransparentColour(gl12, gl12, old_transparent_color_);
	ChangeTransparentColour(gl120, gl120, old_transparent_color_);
    ChangeTransparentColour(gl121, gl121, old_transparent_color_);
    ChangeTransparentColour(gl122, gl122, old_transparent_color_);
    ChangeTransparentColour(gl123, gl123, old_transparent_color_);
	ChangeTransparentColour(gl124, gl124, old_transparent_color_);
    ChangeTransparentColour(gl125, gl125, old_transparent_color_);
    ChangeTransparentColour(gl126, gl126, old_transparent_color_);
    ChangeTransparentColour(gl127, gl127, old_transparent_color_);
	ChangeTransparentColour(gl128, gl128, old_transparent_color_);
    ChangeTransparentColour(gl129, gl129, old_transparent_color_);
    ChangeTransparentColour(gl129Striped, gl129Striped, old_transparent_color_);
    ChangeTransparentColour(gl13, gl13, old_transparent_color_);
	ChangeTransparentColour(gl130, gl130, old_transparent_color_);
    ChangeTransparentColour(gl130Striped, gl130Striped, old_transparent_color_);
    ChangeTransparentColour(gl131, gl131, old_transparent_color_);
    ChangeTransparentColour(gl132, gl132, old_transparent_color_);
	ChangeTransparentColour(gl133, gl133, old_transparent_color_);
    ChangeTransparentColour(gl134, gl134, old_transparent_color_);
    ChangeTransparentColour(gl135, gl135, old_transparent_color_);
    ChangeTransparentColour(gl136, gl136, old_transparent_color_);
	ChangeTransparentColour(gl137, gl137, old_transparent_color_);
    ChangeTransparentColour(gl138, gl138, old_transparent_color_);
    ChangeTransparentColour(gl139, gl139, old_transparent_color_);
    ChangeTransparentColour(gl14, gl14, old_transparent_color_);
	ChangeTransparentColour(gl140, gl140, old_transparent_color_);
    ChangeTransparentColour(gl141, gl141, old_transparent_color_);
    ChangeTransparentColour(gl142, gl142, old_transparent_color_);
    ChangeTransparentColour(gl143, gl143, old_transparent_color_);
	ChangeTransparentColour(gl145, gl145, old_transparent_color_);
    ChangeTransparentColour(gl145Striped, gl145Striped, old_transparent_color_);
    ChangeTransparentColour(gl146, gl146, old_transparent_color_);
    ChangeTransparentColour(gl146Striped, gl146Striped, old_transparent_color_);
	ChangeTransparentColour(gl15, gl15, old_transparent_color_);
    ChangeTransparentColour(gl16, gl16, old_transparent_color_);
    ChangeTransparentColour(gl18, gl18, old_transparent_color_);
    ChangeTransparentColour(gl19, gl19, old_transparent_color_);
	ChangeTransparentColour(gl2, gl2, old_transparent_color_);
    ChangeTransparentColour(gl20, gl20, old_transparent_color_);
    ChangeTransparentColour(gl21, gl21, old_transparent_color_);
    ChangeTransparentColour(gl22, gl22, old_transparent_color_);
	ChangeTransparentColour(gl23, gl23, old_transparent_color_);
    ChangeTransparentColour(gl24, gl24, old_transparent_color_);
    ChangeTransparentColour(gl25, gl25, old_transparent_color_);
    ChangeTransparentColour(gl26, gl26, old_transparent_color_);
	ChangeTransparentColour(gl27, gl27, old_transparent_color_);
    ChangeTransparentColour(gl28, gl28, old_transparent_color_);
    ChangeTransparentColour(gl29, gl29, old_transparent_color_);
    ChangeTransparentColour(gl3, gl3, old_transparent_color_);
	ChangeTransparentColour(gl30, gl30, old_transparent_color_);
    ChangeTransparentColour(gl31, gl31, old_transparent_color_);
    ChangeTransparentColour(gl32, gl32, old_transparent_color_);
    ChangeTransparentColour(gl33, gl33, old_transparent_color_);
	ChangeTransparentColour(gl34, gl34, old_transparent_color_);
    ChangeTransparentColour(gl35, gl35, old_transparent_color_);
    ChangeTransparentColour(gl36, gl36, old_transparent_color_);
    ChangeTransparentColour(gl37, gl37, old_transparent_color_);
	ChangeTransparentColour(gl38, gl38, old_transparent_color_);
    ChangeTransparentColour(gl39, gl39, old_transparent_color_);
    ChangeTransparentColour(gl4, gl4, old_transparent_color_);
    ChangeTransparentColour(gl40, gl40, old_transparent_color_);
	ChangeTransparentColour(gl41, gl41, old_transparent_color_);
    ChangeTransparentColour(gl42, gl42, old_transparent_color_);
    ChangeTransparentColour(gl43, gl43, old_transparent_color_);
    ChangeTransparentColour(gl44, gl44, old_transparent_color_);
	ChangeTransparentColour(gl45, gl45, old_transparent_color_);
    ChangeTransparentColour(gl46, gl46, old_transparent_color_);
    ChangeTransparentColour(gl47, gl47, old_transparent_color_);
    ChangeTransparentColour(gl48, gl48, old_transparent_color_);
	ChangeTransparentColour(gl49, gl49, old_transparent_color_);
    ChangeTransparentColour(gl5, gl5, old_transparent_color_);
    ChangeTransparentColour(gl50, gl50, old_transparent_color_);
    ChangeTransparentColour(gl51, gl51, old_transparent_color_);
	ChangeTransparentColour(gl52, gl52, old_transparent_color_);
    ChangeTransparentColour(gl53, gl53, old_transparent_color_);
    ChangeTransparentColour(gl54, gl54, old_transparent_color_);
    ChangeTransparentColour(gl55, gl55, old_transparent_color_);
	ChangeTransparentColour(gl56, gl56, old_transparent_color_);
    ChangeTransparentColour(gl57, gl57, old_transparent_color_);
    ChangeTransparentColour(gl58, gl58, old_transparent_color_);
    ChangeTransparentColour(gl59, gl59, old_transparent_color_);
	ChangeTransparentColour(gl6, gl6, old_transparent_color_);
    ChangeTransparentColour(gl60, gl60, old_transparent_color_);
    ChangeTransparentColour(gl61, gl61, old_transparent_color_);
    ChangeTransparentColour(gl62, gl62, old_transparent_color_);
	ChangeTransparentColour(gl63, gl63, old_transparent_color_);
    ChangeTransparentColour(gl64, gl64, old_transparent_color_);
    ChangeTransparentColour(gl65, gl65, old_transparent_color_);
    ChangeTransparentColour(gl66, gl66, old_transparent_color_);
	ChangeTransparentColour(gl67, gl67, old_transparent_color_);
    ChangeTransparentColour(gl68, gl68, old_transparent_color_);
    ChangeTransparentColour(gl69, gl69, old_transparent_color_);
    ChangeTransparentColour(gl7, gl7, old_transparent_color_);
	ChangeTransparentColour(gl70, gl70, old_transparent_color_);
    ChangeTransparentColour(gl71, gl71, old_transparent_color_);
    ChangeTransparentColour(gl72, gl72, old_transparent_color_);
    ChangeTransparentColour(gl73, gl73, old_transparent_color_);
	ChangeTransparentColour(gl74, gl74, old_transparent_color_);
    ChangeTransparentColour(gl75, gl75, old_transparent_color_);
    ChangeTransparentColour(gl76, gl76, old_transparent_color_);
    ChangeTransparentColour(gl76Striped, gl76Striped, old_transparent_color_);
    ChangeTransparentColour(gl77, gl77, old_transparent_color_);
	ChangeTransparentColour(gl78, gl78, old_transparent_color_);
    ChangeTransparentColour(gl79, gl79, old_transparent_color_);
    ChangeTransparentColour(gl79Striped, gl79Striped, old_transparent_color_);
    ChangeTransparentColour(gl8, gl8, old_transparent_color_);
    ChangeTransparentColour(gl80, gl80, old_transparent_color_);
    ChangeTransparentColour(gl81, gl81, old_transparent_color_);
	ChangeTransparentColour(gl82, gl82, old_transparent_color_);
    ChangeTransparentColour(gl83, gl83, old_transparent_color_);
    ChangeTransparentColour(gl84, gl84, old_transparent_color_);
    ChangeTransparentColour(gl85, gl85, old_transparent_color_);
    ChangeTransparentColour(gl86, gl86, old_transparent_color_);
    ChangeTransparentColour(gl87, gl87, old_transparent_color_);
	ChangeTransparentColour(gl88set, gl88set, old_transparent_color_);
    ChangeTransparentColour(gl88unset, gl88unset, old_transparent_color_);
    ChangeTransparentColour(gl89set, gl89set, old_transparent_color_);
    ChangeTransparentColour(gl89unset, gl89unset, old_transparent_color_);
    ChangeTransparentColour(gl9, gl9, old_transparent_color_);
    ChangeTransparentColour(gl90set, gl90set, old_transparent_color_);
	ChangeTransparentColour(gl90unset, gl90unset, old_transparent_color_);
    ChangeTransparentColour(gl91set, gl91set, old_transparent_color_);
    ChangeTransparentColour(gl91unset, gl91unset, old_transparent_color_);
    ChangeTransparentColour(gl92set, gl92set, old_transparent_color_);
    ChangeTransparentColour(gl92unset, gl92unset, old_transparent_color_);
    ChangeTransparentColour(gl93set, gl93set, old_transparent_color_);
	ChangeTransparentColour(gl94set, gl94set, old_transparent_color_);
    ChangeTransparentColour(gl95set, gl95set, old_transparent_color_);
    ChangeTransparentColour(gl95unset, gl95unset, old_transparent_color_);
    ChangeTransparentColour(gl97, gl97, old_transparent_color_);
    ChangeTransparentColour(gl98, gl98, old_transparent_color_);
    ChangeTransparentColour(gl99, gl99, old_transparent_color_);
	ChangeTransparentColour(Plat68, Plat68, old_transparent_color_);
    ChangeTransparentColour(Plat68Striped, Plat68Striped, old_transparent_color_);
    ChangeTransparentColour(Plat69, Plat69, old_transparent_color_);
    ChangeTransparentColour(Plat69Striped, Plat69Striped, old_transparent_color_);
    ChangeTransparentColour(Plat70, Plat70, old_transparent_color_);
    ChangeTransparentColour(Plat70Striped, Plat70Striped, old_transparent_color_);
	ChangeTransparentColour(Plat71, Plat71, old_transparent_color_);
    ChangeTransparentColour(Plat71Striped, Plat71Striped, old_transparent_color_);
    ChangeTransparentColour(sm1, sm1, old_transparent_color_);
    ChangeTransparentColour(sm10, sm10, old_transparent_color_);
    ChangeTransparentColour(sm100, sm100, old_transparent_color_);
    ChangeTransparentColour(sm101, sm101, old_transparent_color_);
	ChangeTransparentColour(sm102, sm102, old_transparent_color_);
    ChangeTransparentColour(sm103, sm103, old_transparent_color_);
    ChangeTransparentColour(sm104, sm104, old_transparent_color_);
    ChangeTransparentColour(sm105, sm105, old_transparent_color_);
    ChangeTransparentColour(sm106, sm106, old_transparent_color_);
    ChangeTransparentColour(sm107, sm107, old_transparent_color_);
	ChangeTransparentColour(sm108, sm108, old_transparent_color_);
    ChangeTransparentColour(sm109, sm109, old_transparent_color_);
    ChangeTransparentColour(sm11, sm11, old_transparent_color_);
    ChangeTransparentColour(sm110, sm110, old_transparent_color_);
    ChangeTransparentColour(sm111, sm111, old_transparent_color_);
    ChangeTransparentColour(sm112, sm112, old_transparent_color_);
	ChangeTransparentColour(sm115, sm115, old_transparent_color_);
    ChangeTransparentColour(sm117, sm117, old_transparent_color_);
    ChangeTransparentColour(sm12, sm12, old_transparent_color_);
    ChangeTransparentColour(sm129, sm129, old_transparent_color_);
    ChangeTransparentColour(sm129striped, sm129striped, old_transparent_color_);
    ChangeTransparentColour(sm13, sm13, old_transparent_color_);
	ChangeTransparentColour(sm130, sm130, old_transparent_color_);
    ChangeTransparentColour(sm130striped, sm130striped, old_transparent_color_);
    ChangeTransparentColour(sm131striped, sm131striped, old_transparent_color_);
    ChangeTransparentColour(sm132, sm132, old_transparent_color_);
    ChangeTransparentColour(sm133, sm133, old_transparent_color_);
    ChangeTransparentColour(sm134, sm134, old_transparent_color_);
	ChangeTransparentColour(sm135, sm135, old_transparent_color_);
    ChangeTransparentColour(sm136, sm136, old_transparent_color_);
    ChangeTransparentColour(sm137, sm137, old_transparent_color_);
    ChangeTransparentColour(sm138, sm138, old_transparent_color_);
    ChangeTransparentColour(sm139, sm139, old_transparent_color_);
    ChangeTransparentColour(sm14, sm14, old_transparent_color_);
	ChangeTransparentColour(sm15, sm15, old_transparent_color_);
    ChangeTransparentColour(sm16, sm16, old_transparent_color_);
    ChangeTransparentColour(sm18, sm18, old_transparent_color_);
    ChangeTransparentColour(sm19, sm19, old_transparent_color_);
    ChangeTransparentColour(sm2, sm2, old_transparent_color_);
    ChangeTransparentColour(sm20, sm20, old_transparent_color_);
	ChangeTransparentColour(sm21, sm21, old_transparent_color_);
    ChangeTransparentColour(sm22, sm22, old_transparent_color_);
    ChangeTransparentColour(sm23, sm23, old_transparent_color_);
    ChangeTransparentColour(sm24, sm24, old_transparent_color_);
    ChangeTransparentColour(sm25, sm25, old_transparent_color_);
    ChangeTransparentColour(sm26, sm26, old_transparent_color_);
	ChangeTransparentColour(sm27, sm27, old_transparent_color_);
    ChangeTransparentColour(sm28, sm28, old_transparent_color_);
    ChangeTransparentColour(sm29, sm29, old_transparent_color_);
    ChangeTransparentColour(sm3, sm3, old_transparent_color_);
    ChangeTransparentColour(sm30, sm30, old_transparent_color_);
    ChangeTransparentColour(sm31, sm31, old_transparent_color_);
	ChangeTransparentColour(sm32, sm32, old_transparent_color_);
    ChangeTransparentColour(sm33, sm33, old_transparent_color_);
    ChangeTransparentColour(sm34, sm34, old_transparent_color_);
    ChangeTransparentColour(sm35, sm35, old_transparent_color_);
    ChangeTransparentColour(sm36, sm36, old_transparent_color_);
    ChangeTransparentColour(sm37, sm37, old_transparent_color_);
	ChangeTransparentColour(sm38, sm38, old_transparent_color_);
    ChangeTransparentColour(sm39, sm39, old_transparent_color_);
    ChangeTransparentColour(sm4, sm4, old_transparent_color_);
    ChangeTransparentColour(sm40, sm40, old_transparent_color_);
    ChangeTransparentColour(sm41, sm41, old_transparent_color_);
    ChangeTransparentColour(sm42, sm42, old_transparent_color_);
	ChangeTransparentColour(sm43, sm43, old_transparent_color_);
    ChangeTransparentColour(sm44, sm44, old_transparent_color_);
    ChangeTransparentColour(sm45, sm45, old_transparent_color_);
    ChangeTransparentColour(sm46, sm46, old_transparent_color_);
    ChangeTransparentColour(sm47, sm47, old_transparent_color_);
    ChangeTransparentColour(sm48, sm48, old_transparent_color_);
	ChangeTransparentColour(sm49, sm49, old_transparent_color_);
    ChangeTransparentColour(sm5, sm5, old_transparent_color_);
    ChangeTransparentColour(sm50, sm50, old_transparent_color_);
    ChangeTransparentColour(sm51, sm51, old_transparent_color_);
    ChangeTransparentColour(sm52, sm52, old_transparent_color_);
    ChangeTransparentColour(sm53, sm53, old_transparent_color_);
	ChangeTransparentColour(sm54, sm54, old_transparent_color_);
    ChangeTransparentColour(sm55, sm55, old_transparent_color_);
    ChangeTransparentColour(sm56, sm56, old_transparent_color_);
    ChangeTransparentColour(sm57, sm57, old_transparent_color_);
    ChangeTransparentColour(sm58, sm58, old_transparent_color_);
    ChangeTransparentColour(sm59, sm59, old_transparent_color_);
	ChangeTransparentColour(sm6, sm6, old_transparent_color_);
    ChangeTransparentColour(sm60, sm60, old_transparent_color_);
    ChangeTransparentColour(sm61, sm61, old_transparent_color_);
    ChangeTransparentColour(sm62, sm62, old_transparent_color_);
    ChangeTransparentColour(sm63, sm63, old_transparent_color_);
    ChangeTransparentColour(sm64, sm64, old_transparent_color_);
	ChangeTransparentColour(sm65, sm65, old_transparent_color_);
    ChangeTransparentColour(sm66, sm66, old_transparent_color_);
    ChangeTransparentColour(sm67, sm67, old_transparent_color_);
    ChangeTransparentColour(sm7, sm7, old_transparent_color_);
    ChangeTransparentColour(sm76, sm76, old_transparent_color_);
    ChangeTransparentColour(sm76striped, sm76striped, old_transparent_color_);
	ChangeTransparentColour(sm77, sm77, old_transparent_color_);
    ChangeTransparentColour(sm77striped, sm77striped, old_transparent_color_);
    ChangeTransparentColour(sm78, sm78, old_transparent_color_);
    ChangeTransparentColour(sm78striped, sm78striped, old_transparent_color_);
    ChangeTransparentColour(sm79, sm79, old_transparent_color_);
    ChangeTransparentColour(sm79striped, sm79striped, old_transparent_color_);
	ChangeTransparentColour(sm8, sm8, old_transparent_color_);
    ChangeTransparentColour(sm80, sm80, old_transparent_color_);
    ChangeTransparentColour(sm81, sm81, old_transparent_color_);
    ChangeTransparentColour(sm82, sm82, old_transparent_color_);
    ChangeTransparentColour(sm83, sm83, old_transparent_color_);
    ChangeTransparentColour(sm84, sm84, old_transparent_color_);
	ChangeTransparentColour(sm85, sm85, old_transparent_color_);
    ChangeTransparentColour(sm86, sm86, old_transparent_color_);
    ChangeTransparentColour(sm87, sm87, old_transparent_color_);
    ChangeTransparentColour(sm88, sm88, old_transparent_color_);
    ChangeTransparentColour(sm89, sm89, old_transparent_color_);
    ChangeTransparentColour(sm9, sm9, old_transparent_color_);
	ChangeTransparentColour(sm90, sm90, old_transparent_color_);
    ChangeTransparentColour(sm91, sm91, old_transparent_color_);
    ChangeTransparentColour(sm92, sm92, old_transparent_color_);
    ChangeTransparentColour(sm93, sm93, old_transparent_color_);
    ChangeTransparentColour(sm94, sm94, old_transparent_color_);
    ChangeTransparentColour(sm95, sm95, old_transparent_color_);
	ChangeTransparentColour(sm96, sm96, old_transparent_color_);
    ChangeTransparentColour(sm96striped, sm96striped, old_transparent_color_);
    ChangeTransparentColour(sm97, sm97, old_transparent_color_);
    ChangeTransparentColour(sm98, sm98, old_transparent_color_);
    ChangeTransparentColour(sm99, sm99, old_transparent_color_);
    ChangeTransparentColour(smBlack, smBlack, old_transparent_color_);
	ChangeTransparentColour(smBlue, smBlue, old_transparent_color_);
    ChangeTransparentColour(smBrightGreen, smBrightGreen, old_transparent_color_);
    ChangeTransparentColour(smCaramel, smCaramel, old_transparent_color_);
    ChangeTransparentColour(smCyan, smCyan, old_transparent_color_);
    ChangeTransparentColour(smLightBlue, smLightBlue, old_transparent_color_);
    ChangeTransparentColour(smMagenta, smMagenta, old_transparent_color_);
	ChangeTransparentColour(smName, smName, old_transparent_color_);
    ChangeTransparentColour(smPaleGreen, smPaleGreen, old_transparent_color_);
    ChangeTransparentColour(smRed, smRed, old_transparent_color_);
    ChangeTransparentColour(smYellow, smYellow, old_transparent_color_);
    ChangeTransparentColour(smTransparent, smTransparent, old_transparent_color_);
    ChangeTransparentColour(TempBackground, TempBackground, old_transparent_color_);
	ChangeTransparentColour(TempHeadCode, TempHeadCode, old_transparent_color_);
    ChangeTransparentColour(UnderHFootbridge, UnderHFootbridge, old_transparent_color_);
    ChangeTransparentColour(UnderVFootbridge, UnderVFootbridge, old_transparent_color_);

    ChangeTransparentColour(FSig68, FSig68, old_transparent_color_); //failed signals, added at v2.13.0
    ChangeTransparentColour(FSig69, FSig69, old_transparent_color_);
	ChangeTransparentColour(FSig70, FSig70, old_transparent_color_);
	ChangeTransparentColour(FSig71, FSig71, old_transparent_color_);
    ChangeTransparentColour(FSig72, FSig72, old_transparent_color_);
    ChangeTransparentColour(FSig73, FSig73, old_transparent_color_);
    ChangeTransparentColour(FSig74, FSig74, old_transparent_color_);
    ChangeTransparentColour(FSig75, FSig75, old_transparent_color_);
	ChangeTransparentColour(FGSig68, FGSig68, old_transparent_color_);
    ChangeTransparentColour(FGSig69, FGSig69, old_transparent_color_);
    ChangeTransparentColour(FGSig70, FGSig70, old_transparent_color_);
    ChangeTransparentColour(FGSig71, FGSig71, old_transparent_color_);
    ChangeTransparentColour(FGSig72, FGSig72, old_transparent_color_);
    ChangeTransparentColour(FGSig73, FGSig73, old_transparent_color_);
	ChangeTransparentColour(FGSig74, FGSig74, old_transparent_color_);
    ChangeTransparentColour(FGSig75, FGSig75, old_transparent_color_);

    // The following are created as new bitmaps from existing  files
	ChangeTransparentColour(bmTransparentBgnd, bmTransparentBgnd, old_transparent_color_);
    ChangeTransparentColour(GridBitmap, GridBitmap, old_transparent_color_);

    // non-transparent graphics - don't change headcodes
    ChangeTransparentColour(bmSolidBgnd, bmSolidBgnd, old_transparent_color_);
    ChangeTransparentColour(smSolidBgnd, smSolidBgnd, old_transparent_color_);
    ChangeTransparentColour(bmDiagonalSignalBlank, bmDiagonalSignalBlank, old_transparent_color_);
    ChangeTransparentColour(bmPointBlank, bmPointBlank, old_transparent_color_);
	ChangeTransparentColour(bmStraightEWSignalBlank, bmStraightEWSignalBlank, old_transparent_color_);
    ChangeTransparentColour(bmStraightNSSignalBlank, bmStraightNSSignalBlank, old_transparent_color_);

    // level crossing graphics
    ChangeTransparentColour(LCBothHor, LCBothHor, old_transparent_color_);
    ChangeTransparentColour(LCBotHor, LCBotHor, old_transparent_color_);
	ChangeTransparentColour(LCBothVer, LCBothVer, old_transparent_color_);
    ChangeTransparentColour(LCLHSVer, LCLHSVer, old_transparent_color_);
    ChangeTransparentColour(LCPlain, LCPlain, old_transparent_color_);
    ChangeTransparentColour(LCRHSVer, LCRHSVer, old_transparent_color_);
    ChangeTransparentColour(LCTopHor, LCTopHor, old_transparent_color_);
    ChangeTransparentColour(LCBothHorMan, LCBothHorMan, old_transparent_color_);
	ChangeTransparentColour(LCBotHorMan, LCBotHorMan, old_transparent_color_);
    ChangeTransparentColour(LCBothVerMan, LCBothVerMan, old_transparent_color_);
    ChangeTransparentColour(LCLHSVerMan, LCLHSVerMan, old_transparent_color_);
    ChangeTransparentColour(LCPlainMan, LCPlainMan, old_transparent_color_);
    ChangeTransparentColour(LCRHSVerMan, LCRHSVerMan, old_transparent_color_);
    ChangeTransparentColour(LCTopHorMan, LCTopHorMan, old_transparent_color_);

// change the grid to the nearest grey colour to the background
	if(Utilities->clTransparent != clB5G5R5)
	{
		ChangeSpecificColour(1, GridBitmap, GridBitmap, clB4G4R4, clB1G1R1); // if already dark will ignore
	}
	else
	{
		ChangeSpecificColour(3, GridBitmap, GridBitmap, clB1G1R1, clB4G4R4); // if already light will ignore
	}
}

// ---------------------------------------------------------------------------

void TRailGraphics::SetUpAllDerivativeGraphics()
{
	for(int x = 0; x < 30; x++)
	{
		LinkPrefDirGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
		LinkNonSigRouteGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
		LinkSigRouteGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
		LinkRouteAutoSigsGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
    }
    for(int x = 0; x < 12; x++)
    {
		BridgePrefDirGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
		BridgeNonSigRouteGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
		BridgeSigRouteGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
        BridgeRouteAutoSigsGraphicsPtr[x]->TransparentColor = Utilities->clTransparent;
    }

/*
      EXArray with associated Tag numbers:-
      {4,6} 1,{2,8} 2,                                                         //horizontal & vertical
      {2,4} 6,{6,2} 5,{8,6} 3,{4,8}, 4                                         //sharp curves
      {1,6} 22,{3,8} 24,{9,4} 21,{7,2} 27,{1,8} 25,{3,4} 23,{9,2} 26,{7,6}, 20 //loose curves
      {1,9} 19,{3,7} 18,                                                       //forward & reverse diagonals

      int Bridge EXArray with Tag numbers:-
      {4,6} 48,{2,8} 49,{1,9} 50,{3,7} 51,
      {1,9} 52,{3,7} 56,{1,9} 57,{3,7} 53,
      {2,8} 54,{4,6} 59,{2,8} 55,{4,6} 58

      reverse direction same in each case
*/

    // Note: DirectionPrefDirGraphicsPtr[0] & [5] unused
    DirectionPrefDirGraphicsPtr[0]->Assign(ELk1);
    DirectionPrefDirGraphicsPtr[1]->Assign(ELk1);
    DirectionPrefDirGraphicsPtr[2]->Assign(ELk2);
    DirectionPrefDirGraphicsPtr[3]->Assign(ELk3);
    DirectionPrefDirGraphicsPtr[4]->Assign(ELk4);
    DirectionPrefDirGraphicsPtr[5]->Assign(ELk4);
    DirectionPrefDirGraphicsPtr[6]->Assign(ELk6);
    DirectionPrefDirGraphicsPtr[7]->Assign(ELk7);
    DirectionPrefDirGraphicsPtr[8]->Assign(ELk8);
    DirectionPrefDirGraphicsPtr[9]->Assign(ELk9);

    DirectionNonSigRouteGraphicsPtr[0]->Assign(ELk1);
    DirectionNonSigRouteGraphicsPtr[1]->Assign(ELk1);
    DirectionNonSigRouteGraphicsPtr[2]->Assign(ELk2);
    DirectionNonSigRouteGraphicsPtr[3]->Assign(ELk3);
    DirectionNonSigRouteGraphicsPtr[4]->Assign(ELk4);
    DirectionNonSigRouteGraphicsPtr[5]->Assign(ELk4);
    DirectionNonSigRouteGraphicsPtr[6]->Assign(ELk6);
    DirectionNonSigRouteGraphicsPtr[7]->Assign(ELk7);
    DirectionNonSigRouteGraphicsPtr[8]->Assign(ELk8);
    DirectionNonSigRouteGraphicsPtr[9]->Assign(ELk9);

    DirectionSigRouteGraphicsPtr[0]->Assign(ELk1);
    DirectionSigRouteGraphicsPtr[1]->Assign(ELk1);
    DirectionSigRouteGraphicsPtr[2]->Assign(ELk2);
    DirectionSigRouteGraphicsPtr[3]->Assign(ELk3);
    DirectionSigRouteGraphicsPtr[4]->Assign(ELk4);
    DirectionSigRouteGraphicsPtr[5]->Assign(ELk4);
    DirectionSigRouteGraphicsPtr[6]->Assign(ELk6);
    DirectionSigRouteGraphicsPtr[7]->Assign(ELk7);
    DirectionSigRouteGraphicsPtr[8]->Assign(ELk8);
    DirectionSigRouteGraphicsPtr[9]->Assign(ELk9);

    DirectionRouteAutoSigsGraphicsPtr[0]->Assign(ELk1);
    DirectionRouteAutoSigsGraphicsPtr[1]->Assign(ELk1);
    DirectionRouteAutoSigsGraphicsPtr[2]->Assign(ELk2);
    DirectionRouteAutoSigsGraphicsPtr[3]->Assign(ELk3);
    DirectionRouteAutoSigsGraphicsPtr[4]->Assign(ELk4);
    DirectionRouteAutoSigsGraphicsPtr[5]->Assign(ELk4);
    DirectionRouteAutoSigsGraphicsPtr[6]->Assign(ELk6);
    DirectionRouteAutoSigsGraphicsPtr[7]->Assign(ELk7);
    DirectionRouteAutoSigsGraphicsPtr[8]->Assign(ELk8);
    DirectionRouteAutoSigsGraphicsPtr[9]->Assign(ELk9);

    for(int x = 0; x < 30; x++)
    {
		ChangeForegroundColour(5, LinkGraphicsPtr[x], LinkPrefDirGraphicsPtr[x], clB2G0R4, Utilities->clTransparent); // magenta
		ChangeForegroundColour(6, LinkGraphicsPtr[x], LinkNonSigRouteGraphicsPtr[x], clB0G0R5, Utilities->clTransparent); // red
		ChangeForegroundColour(7, LinkGraphicsPtr[x], LinkSigRouteGraphicsPtr[x], clB0G4R0, Utilities->clTransparent); // green
		ChangeForegroundColour(8, LinkGraphicsPtr[x], LinkRouteAutoSigsGraphicsPtr[x], clB5G3R0, Utilities->clTransparent); // blue
    }
    for(int x = 0; x < 12; x++)
    {
		ChangeForegroundColour(9, BridgeGraphicsPtr[x], BridgePrefDirGraphicsPtr[x], clB2G0R4, Utilities->clTransparent);
		ChangeForegroundColour(10, BridgeGraphicsPtr[x], BridgeNonSigRouteGraphicsPtr[x], clB0G0R5, Utilities->clTransparent);
		ChangeForegroundColour(11, BridgeGraphicsPtr[x], BridgeSigRouteGraphicsPtr[x], clB0G4R0, Utilities->clTransparent);
		ChangeForegroundColour(12, BridgeGraphicsPtr[x], BridgeRouteAutoSigsGraphicsPtr[x], clB5G3R0, Utilities->clTransparent);
    }
    for(int x = 0; x < 10; x++)
    {
		ChangeForegroundColour(13, DirectionPrefDirGraphicsPtr[x], DirectionPrefDirGraphicsPtr[x], clB2G0R4, Utilities->clTransparent);
		ChangeForegroundColour(14, DirectionNonSigRouteGraphicsPtr[x], DirectionNonSigRouteGraphicsPtr[x], clB0G0R5, Utilities->clTransparent);
		ChangeForegroundColour(15, DirectionSigRouteGraphicsPtr[x], DirectionSigRouteGraphicsPtr[x], clB0G4R0, Utilities->clTransparent);
        ChangeForegroundColour(16, DirectionRouteAutoSigsGraphicsPtr[x], DirectionRouteAutoSigsGraphicsPtr[x], clB5G3R0, Utilities->clTransparent);
    }

    // set up points' fillets
    PointModeGraphicsPtr[0][0]->Assign(bm7Straight);
    PointModeGraphicsPtr[1][0]->Assign(bm8Straight);
    PointModeGraphicsPtr[2][0]->Assign(bm9Straight);
    PointModeGraphicsPtr[3][0]->Assign(bm10Straight);
    PointModeGraphicsPtr[4][0]->Assign(bm11Straight);
    PointModeGraphicsPtr[5][0]->Assign(bm12Straight);
    PointModeGraphicsPtr[6][0]->Assign(bm13Straight);
    PointModeGraphicsPtr[7][0]->Assign(bm14Straight);
    PointModeGraphicsPtr[8][0]->Assign(bm28Straight);
    PointModeGraphicsPtr[9][0]->Assign(bm29Straight);
    PointModeGraphicsPtr[10][0]->Assign(bm30Straight);
    PointModeGraphicsPtr[11][0]->Assign(bm31Straight);
    PointModeGraphicsPtr[12][0]->Assign(bm32Straight);
    PointModeGraphicsPtr[13][0]->Assign(bm33Straight);
    PointModeGraphicsPtr[14][0]->Assign(bm34Straight);
    PointModeGraphicsPtr[15][0]->Assign(bm35Straight);
    PointModeGraphicsPtr[16][0]->Assign(bm36Straight);
    PointModeGraphicsPtr[17][0]->Assign(bm37Straight);
    PointModeGraphicsPtr[18][0]->Assign(bm38Straight);
    PointModeGraphicsPtr[19][0]->Assign(bm39Straight);
    PointModeGraphicsPtr[20][0]->Assign(bm40Straight);
    PointModeGraphicsPtr[21][0]->Assign(bm41Straight);
    PointModeGraphicsPtr[22][0]->Assign(bm42Straight);
    PointModeGraphicsPtr[23][0]->Assign(bm43Straight);

    PointModeGraphicsPtr[24][0]->Assign(bm132LeftFork);
    PointModeGraphicsPtr[25][0]->Assign(bm133LeftFork);
    PointModeGraphicsPtr[26][0]->Assign(bm134LeftFork);
    PointModeGraphicsPtr[27][0]->Assign(bm135LeftFork);
    PointModeGraphicsPtr[28][0]->Assign(bm136LeftFork);
    PointModeGraphicsPtr[29][0]->Assign(bm137LeftFork);
    PointModeGraphicsPtr[30][0]->Assign(bm138LeftFork);
    PointModeGraphicsPtr[31][0]->Assign(bm139LeftFork);

    PointModeGraphicsPtr[0][1]->Assign(bm7Diverging);
    PointModeGraphicsPtr[1][1]->Assign(bm8Diverging);
    PointModeGraphicsPtr[2][1]->Assign(bm9Diverging);
    PointModeGraphicsPtr[3][1]->Assign(bm10Diverging);
    PointModeGraphicsPtr[4][1]->Assign(bm11Diverging);
    PointModeGraphicsPtr[5][1]->Assign(bm12Diverging);
    PointModeGraphicsPtr[6][1]->Assign(bm13Diverging);
    PointModeGraphicsPtr[7][1]->Assign(bm14Diverging);
    PointModeGraphicsPtr[8][1]->Assign(bm28Diverging);
    PointModeGraphicsPtr[9][1]->Assign(bm29Diverging);
    PointModeGraphicsPtr[10][1]->Assign(bm30Diverging);
    PointModeGraphicsPtr[11][1]->Assign(bm31Diverging);
    PointModeGraphicsPtr[12][1]->Assign(bm32Diverging);
    PointModeGraphicsPtr[13][1]->Assign(bm33Diverging);
    PointModeGraphicsPtr[14][1]->Assign(bm34Diverging);
    PointModeGraphicsPtr[15][1]->Assign(bm35Diverging);
    PointModeGraphicsPtr[16][1]->Assign(bm36Diverging);
    PointModeGraphicsPtr[17][1]->Assign(bm37Diverging);
    PointModeGraphicsPtr[18][1]->Assign(bm38Diverging);
    PointModeGraphicsPtr[19][1]->Assign(bm39Diverging);
    PointModeGraphicsPtr[20][1]->Assign(bm40Diverging);
    PointModeGraphicsPtr[21][1]->Assign(bm41Diverging);
    PointModeGraphicsPtr[22][1]->Assign(bm42Diverging);
    PointModeGraphicsPtr[23][1]->Assign(bm43Diverging);

    PointModeGraphicsPtr[24][1]->Assign(bm132RightFork);
    PointModeGraphicsPtr[25][1]->Assign(bm133RightFork);
    PointModeGraphicsPtr[26][1]->Assign(bm134RightFork);
    PointModeGraphicsPtr[27][1]->Assign(bm135RightFork);
    PointModeGraphicsPtr[28][1]->Assign(bm136RightFork);
    PointModeGraphicsPtr[29][1]->Assign(bm137RightFork);
    PointModeGraphicsPtr[30][1]->Assign(bm138RightFork);
    PointModeGraphicsPtr[31][1]->Assign(bm139RightFork);
}

// ---------------------------------------------------------------------------

void TRailGraphics::ConvertSignalsToOppositeHand(int Caller) // new at v2.3.0
//no need for new graphic elements, just convert the originals
{
    Utilities->EventLog.push_back("ConvertSignalsToOppositeHand");
    if(Utilities->EventLog.size() > 1000)
    {
        Utilities->EventLog.pop_front();
    }
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ", ConvertSignalsToOppositeHand");

    Graphics::TBitmap* HorizSignalArray[22] =
    {
        Plat68, Plat68Striped, Plat69, Plat69Striped, bm68CallingOn, bm68dblyellow, bm68grounddblred, bm68grounddblwhite, bm68green, bm68yellow, bm69CallingOn,
        bm69dblyellow, bm69grounddblred, bm69grounddblwhite, bm69green, bm69yellow, gl68, gl69, FSig68, FSig69, FGSig68, FGSig69
    };

    Graphics::TBitmap* VertSignalArray[22] =
    {
        Plat70, Plat70Striped, Plat71, Plat71Striped, bm70CallingOn, bm70dblyellow, bm70grounddblred, bm70grounddblwhite, bm70green, bm70yellow, bm71CallingOn,
        bm71dblyellow, bm71grounddblred, bm71grounddblwhite, bm71green, bm71yellow, gl70, gl71, FSig70, FSig71, FGSig70, FGSig71
    };

    Graphics::TBitmap* BackDiagSignalArray[18] =
    {
        bm72CallingOn, bm72dblyellow, bm72grounddblred, bm72grounddblwhite, bm72green, bm72yellow, bm75CallingOn, bm75dblyellow, bm75grounddblred,
        bm75grounddblwhite, bm75green, bm75yellow, gl72, gl75, FSig72, FSig75, FGSig72, FGSig75
    };

    Graphics::TBitmap* FwdDiagSignalArray[20] =
    {
        bm73, bm73CallingOn, bm73dblyellow, bm73grounddblred, bm73grounddblwhite, bm73green, bm73yellow, bm74, bm74CallingOn, bm74dblyellow, bm74grounddblred,
        bm74grounddblwhite, bm74green, bm74yellow, gl73, gl74, FSig73, FSig73, FGSig74, FGSig74
    };

// the following are the stay black glyphs for the speedbuttons
    Graphics::TBitmap* HorizSignalGlyphArray[4] =
    {
        SpeedBut68NormBlackGlyph, SpeedBut69NormBlackGlyph, SpeedBut68GrndBlackGlyph, SpeedBut69GrndBlackGlyph
    };

    Graphics::TBitmap* VertSignalGlyphArray[4] =
    {
        SpeedBut70NormBlackGlyph, SpeedBut71NormBlackGlyph, SpeedBut70GrndBlackGlyph, SpeedBut71GrndBlackGlyph
    };

    Graphics::TBitmap* BackDiagSignalGlyphArray[4] =
    {
        SpeedBut72NormBlackGlyph, SpeedBut75NormBlackGlyph, SpeedBut72GrndBlackGlyph, SpeedBut75GrndBlackGlyph,
    };

    Graphics::TBitmap* FwdDiagSignalGlyphArray[4] =
    {
        SpeedBut73NormBlackGlyph, SpeedBut74NormBlackGlyph, SpeedBut73GrndBlackGlyph, SpeedBut74GrndBlackGlyph
    };

    Graphics::TBitmap* TmpBM;

    TmpBM = new Graphics::TBitmap;
	TmpBM->Assign(bmTransparentBgnd); // current background colour as transparent colour

    for(int x = 0; x < 22; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = HorizSignalArray[x]->Canvas->Pixels[i][15 - j];
            }
        }
        HorizSignalArray[x]->Assign(TmpBM);
    }

    TmpBM->Assign(bmTransparentBgnd); // current background colour as transparent colour
    for(int x = 0; x < 22; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = VertSignalArray[x]->Canvas->Pixels[15 - i][j];
            }
        }
        VertSignalArray[x]->Assign(TmpBM);
    }

	TmpBM->Assign(bmTransparentBgnd); // current background colour as transparent colour
    for(int x = 0; x < 18; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = BackDiagSignalArray[x]->Canvas->Pixels[j][i];
            }
        }
        BackDiagSignalArray[x]->Assign(TmpBM); // current background colour as transparent colour
    }

	TmpBM->Assign(bmTransparentBgnd);
    for(int x = 0; x < 20; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = FwdDiagSignalArray[x]->Canvas->Pixels[15 - j][15 - i];
            }
        }
        FwdDiagSignalArray[x]->Assign(TmpBM);
    }

    Modifier->load_graphic(TmpBM, "bmSolidBgnd");
    TmpBM->Transparent = true;
    TmpBM->TransparentColor = clB5G5R5;
    // white as transparent colour for speedbutton glyphs
    for(int x = 0; x < 4; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = HorizSignalGlyphArray[x]->Canvas->Pixels[i][15 - j];
            }
        }
        HorizSignalGlyphArray[x]->Assign(TmpBM);
    }

    Modifier->load_graphic(TmpBM, "bmSolidBgnd");
    TmpBM->Transparent = true;
    TmpBM->TransparentColor = clB5G5R5;
    // white as transparent colour for speedbutton glyphs
    for(int x = 0; x < 4; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = VertSignalGlyphArray[x]->Canvas->Pixels[15 - i][j];
            }
        }
        VertSignalGlyphArray[x]->Assign(TmpBM);
    }

    Modifier->load_graphic(TmpBM, "bmSolidBgnd");
    TmpBM->Transparent = true;
    TmpBM->TransparentColor = clB5G5R5;
    // white as transparent colour for speedbutton glyphs
    for(int x = 0; x < 4; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = BackDiagSignalGlyphArray[x]->Canvas->Pixels[j][i];
            }
        }
        BackDiagSignalGlyphArray[x]->Assign(TmpBM);
    }

	Modifier->load_graphic(TmpBM, "bmSolidBgnd");
    TmpBM->Transparent = true;
    TmpBM->TransparentColor = clB5G5R5;
    // white as transparent colour for speedbutton glyphs
    for(int x = 0; x < 4; x++)
    {
        for(int i = 0; i < 16; i++)
        {
            for(int j = 0; j < 16; j++)
            {
                TmpBM->Canvas->Pixels[i][j] = FwdDiagSignalGlyphArray[x]->Canvas->Pixels[15 - j][15 - i];
            }
        }
        FwdDiagSignalGlyphArray[x]->Assign(TmpBM);
    }

    Utilities->RHSignalFlag = !Utilities->RHSignalFlag; // set in itially to false (= LH)
    delete TmpBM;
    Utilities->CallLogPop(74);
}

// ---------------------------------------------------------------------------

//function to generate RGB heatmap values from an input value between 0 and 1 using 7 colours
//derived from code created by Dr Andrew Noske (https://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients) with
//his permission to use or adapt.

void TRailGraphics::GetHeatMapColor(int Caller, float value, int *red, int *green, int *blue)
{
    Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + "," + AnsiString(value) + ", GetHeatMapColor");
    const int NUM_COLORS = 7;
//  static int color[NUM_COLORS][3] = { {0,0,255}, {0,255,255}, {0,255,0}, {255,255,0}, {255,0,255}, {255,0,0} };
    // A static array of 6 colors:  (blue, cyan, green, yellow, magenta, red) using {r,g,b} for each.
    static int color[NUM_COLORS][3] = { {255,0,0}, {255,127,0}, {255,255,0}, {0,255,0}, {0,255,255}, {0,0,255}, {127,0,255} };
    // A static array of 7 (rainbow) colors:  (red, orange, yellow, green, cyan, blue, voilet) using {r,g,b} for each.

    int idx1;        // |-- Our desired color will be between these two indexes in "color".
    int idx2;        // |
    float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

    if(value <= 0)       {  idx1 = idx2 = 0;            }    // accounts for an input <=0
    else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=1
    else
    {
        value = value * (NUM_COLORS-1);        // Will multiply value by 5.
        idx1  = floor(value);                  // Our desired color will be after this index.
        idx2  = idx1 + 1;                        // ... and before this index (inclusive).
        fractBetween = value - float(idx1);           // Distance between the two indexes (0-1).
    }

    *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
    *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
    *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
    Utilities->CallLogPop(2717);
}

// ---------------------------------------------------------------------------


