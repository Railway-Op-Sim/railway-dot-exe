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

// ---------------------------------------------------------------------------
#pragma package(smart_init)
// ---------------------------------------------------------------------------

TRailGraphics *RailGraphics;

// ---------------------------------------------------------------------------

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

    loadGraphics();


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

}
// ---------------------------------------------------------------------------

TRailGraphics::~TRailGraphics()
{
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
    loadResourceFromPrefix(bm10, "bm10", RESOURCE_PREFIX);
    bm10->Transparent = true;
    bm10->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(BlackCircle, "BlackCircle", RESOURCE_PREFIX);
    BlackCircle->Transparent = true;
    BlackCircle->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm100, "bm100", RESOURCE_PREFIX);
    bm100->Transparent = true;
    bm100->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm101, "bm101", RESOURCE_PREFIX);
    bm101->Transparent = true;
    bm101->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm106, "bm106", RESOURCE_PREFIX);
    bm106->Transparent = true;
    bm106->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm10Diverging, "bm10Diverging", RESOURCE_PREFIX);
    bm10Diverging->Transparent = true;
    bm10Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm10Straight, "bm10Straight", RESOURCE_PREFIX);
    bm10Straight->Transparent = true;
    bm10Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm11, "bm11", RESOURCE_PREFIX);
    bm11->Transparent = true;
    bm11->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm11Diverging, "bm11Diverging", RESOURCE_PREFIX);
    bm11Diverging->Transparent = true;
    bm11Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm11Straight, "bm11Straight", RESOURCE_PREFIX);
    bm11Straight->Transparent = true;
    bm11Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm12, "bm12", RESOURCE_PREFIX);
    bm12->Transparent = true;
    bm12->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm12Diverging, "bm12Diverging", RESOURCE_PREFIX);
    bm12Diverging->Transparent = true;
    bm12Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm12Straight, "bm12Straight", RESOURCE_PREFIX);
    bm12Straight->Transparent = true;
    bm12Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm13, "bm13", RESOURCE_PREFIX);
    bm13->Transparent = true;
    bm13->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm132, "bm132", RESOURCE_PREFIX);
    bm132->Transparent = true;
    bm132->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm132LeftFork, "bm132LeftFork", RESOURCE_PREFIX);
    bm132LeftFork->Transparent = true;
    bm132LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm132RightFork, "bm132RightFork", RESOURCE_PREFIX);
    bm132RightFork->Transparent = true;
    bm132RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm133, "bm133", RESOURCE_PREFIX);
    bm133->Transparent = true;
    bm133->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm133LeftFork, "bm133LeftFork", RESOURCE_PREFIX);
    bm133LeftFork->Transparent = true;
    bm133LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm133RightFork, "bm133RightFork", RESOURCE_PREFIX);
    bm133RightFork->Transparent = true;
    bm133RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm134, "bm134", RESOURCE_PREFIX);
    bm134->Transparent = true;
    bm134->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm134LeftFork, "bm134LeftFork", RESOURCE_PREFIX);
    bm134LeftFork->Transparent = true;
    bm134LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm134RightFork, "bm134RightFork", RESOURCE_PREFIX);
    bm134RightFork->Transparent = true;
    bm134RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm135, "bm135", RESOURCE_PREFIX);
    bm135->Transparent = true;
    bm135->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm135LeftFork, "bm135LeftFork", RESOURCE_PREFIX);
    bm135LeftFork->Transparent = true;
    bm135LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm135RightFork, "bm135RightFork", RESOURCE_PREFIX);
    bm135RightFork->Transparent = true;
    bm135RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm136, "bm136", RESOURCE_PREFIX);
    bm136->Transparent = true;
    bm136->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm136LeftFork, "bm136LeftFork", RESOURCE_PREFIX);
    bm136LeftFork->Transparent = true;
    bm136LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm136RightFork, "bm136RightFork", RESOURCE_PREFIX);
    bm136RightFork->Transparent = true;
    bm136RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm137, "bm137", RESOURCE_PREFIX);
    bm137->Transparent = true;
    bm137->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm137LeftFork, "bm137LeftFork", RESOURCE_PREFIX);
    bm137LeftFork->Transparent = true;
    bm137LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm137RightFork, "bm137RightFork", RESOURCE_PREFIX);
    bm137RightFork->Transparent = true;
    bm137RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm138, "bm138", RESOURCE_PREFIX);
    bm138->Transparent = true;
    bm138->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm138LeftFork, "bm138LeftFork", RESOURCE_PREFIX);
    bm138LeftFork->Transparent = true;
    bm138LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm138RightFork, "bm138RightFork", RESOURCE_PREFIX);
    bm138RightFork->Transparent = true;
    bm138RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm139, "bm139", RESOURCE_PREFIX);
    bm139->Transparent = true;
    bm139->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm139LeftFork, "bm139LeftFork", RESOURCE_PREFIX);
    bm139LeftFork->Transparent = true;
    bm139LeftFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm139RightFork, "bm139RightFork", RESOURCE_PREFIX);
    bm139RightFork->Transparent = true;
    bm139RightFork->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm13Diverging, "bm13Diverging", RESOURCE_PREFIX);
    bm13Diverging->Transparent = true;
    bm13Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm13Straight, "bm13Straight", RESOURCE_PREFIX);
    bm13Straight->Transparent = true;
    bm13Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm14, "bm14", RESOURCE_PREFIX);
    bm14->Transparent = true;
    bm14->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm140, "bm140", RESOURCE_PREFIX);
    bm140->Transparent = true;
    bm140->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm141, "bm141", RESOURCE_PREFIX);
    bm141->Transparent = true;
    bm141->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm14Diverging, "bm14Diverging", RESOURCE_PREFIX);
    bm14Diverging->Transparent = true;
    bm14Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm14Straight, "bm14Straight", RESOURCE_PREFIX);
    bm14Straight->Transparent = true;
    bm14Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm16, "bm16", RESOURCE_PREFIX);
    bm16->Transparent = true;
    bm16->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm18, "bm18", RESOURCE_PREFIX);
    bm18->Transparent = true;
    bm18->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm20, "bm20", RESOURCE_PREFIX);
    bm20->Transparent = true;
    bm20->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm27, "bm27", RESOURCE_PREFIX);
    bm27->Transparent = true;
    bm27->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm28, "bm28", RESOURCE_PREFIX);
    bm28->Transparent = true;
    bm28->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm28Diverging, "bm28Diverging", RESOURCE_PREFIX);
    bm28Diverging->Transparent = true;
    bm28Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm28Straight, "bm28Straight", RESOURCE_PREFIX);
    bm28Straight->Transparent = true;
    bm28Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm29, "bm29", RESOURCE_PREFIX);
    bm29->Transparent = true;
    bm29->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm29Diverging, "bm29Diverging", RESOURCE_PREFIX);
    bm29Diverging->Transparent = true;
    bm29Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm29Straight, "bm29Straight", RESOURCE_PREFIX);
    bm29Straight->Transparent = true;
    bm29Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm30, "bm30", RESOURCE_PREFIX);
    bm30->Transparent = true;
    bm30->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm30Diverging, "bm30Diverging", RESOURCE_PREFIX);
    bm30Diverging->Transparent = true;
    bm30Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm30Straight, "bm30Straight", RESOURCE_PREFIX);
    bm30Straight->Transparent = true;
    bm30Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm31, "bm31", RESOURCE_PREFIX);
    bm31->Transparent = true;
    bm31->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm31Diverging, "bm31Diverging", RESOURCE_PREFIX);
    bm31Diverging->Transparent = true;
    bm31Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm31Straight, "bm31Straight", RESOURCE_PREFIX);
    bm31Straight->Transparent = true;
    bm31Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm32, "bm32", RESOURCE_PREFIX);
    bm32->Transparent = true;
    bm32->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm32Diverging, "bm32Diverging", RESOURCE_PREFIX);
    bm32Diverging->Transparent = true;
    bm32Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm32Straight, "bm32Straight", RESOURCE_PREFIX);
    bm32Straight->Transparent = true;
    bm32Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm33, "bm33", RESOURCE_PREFIX);
    bm33->Transparent = true;
    bm33->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm33Diverging, "bm33Diverging", RESOURCE_PREFIX);
    bm33Diverging->Transparent = true;
    bm33Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm33Straight, "bm33Straight", RESOURCE_PREFIX);
    bm33Straight->Transparent = true;
    bm33Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm34, "bm34", RESOURCE_PREFIX);
    bm34->Transparent = true;
    bm34->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm34Diverging, "bm34Diverging", RESOURCE_PREFIX);
    bm34Diverging->Transparent = true;
    bm34Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm34Straight, "bm34Straight", RESOURCE_PREFIX);
    bm34Straight->Transparent = true;
    bm34Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm35, "bm35", RESOURCE_PREFIX);
    bm35->Transparent = true;
    bm35->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm35Diverging, "bm35Diverging", RESOURCE_PREFIX);
    bm35Diverging->Transparent = true;
    bm35Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm35Straight, "bm35Straight", RESOURCE_PREFIX);
    bm35Straight->Transparent = true;
    bm35Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm36, "bm36", RESOURCE_PREFIX);
    bm36->Transparent = true;
    bm36->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm36Diverging, "bm36Diverging", RESOURCE_PREFIX);
    bm36Diverging->Transparent = true;
    bm36Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm36Straight, "bm36Straight", RESOURCE_PREFIX);
    bm36Straight->Transparent = true;
    bm36Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm37, "bm37", RESOURCE_PREFIX);
    bm37->Transparent = true;
    bm37->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm37Diverging, "bm37Diverging", RESOURCE_PREFIX);
    bm37Diverging->Transparent = true;
    bm37Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm37Straight, "bm37Straight", RESOURCE_PREFIX);
    bm37Straight->Transparent = true;
    bm37Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm38, "bm38", RESOURCE_PREFIX);
    bm38->Transparent = true;
    bm38->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm38Diverging, "bm38Diverging", RESOURCE_PREFIX);
    bm38Diverging->Transparent = true;
    bm38Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm38Straight, "bm38Straight", RESOURCE_PREFIX);
    bm38Straight->Transparent = true;
    bm38Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm39, "bm39", RESOURCE_PREFIX);
    bm39->Transparent = true;
    bm39->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm39Diverging, "bm39Diverging", RESOURCE_PREFIX);
    bm39Diverging->Transparent = true;
    bm39Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm39Straight, "bm39Straight", RESOURCE_PREFIX);
    bm39Straight->Transparent = true;
    bm39Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm40, "bm40", RESOURCE_PREFIX);
    bm40->Transparent = true;
    bm40->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm40Diverging, "bm40Diverging", RESOURCE_PREFIX);
    bm40Diverging->Transparent = true;
    bm40Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm40Straight, "bm40Straight", RESOURCE_PREFIX);
    bm40Straight->Transparent = true;
    bm40Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm41, "bm41", RESOURCE_PREFIX);
    bm41->Transparent = true;
    bm41->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm41Diverging, "bm41Diverging", RESOURCE_PREFIX);
    bm41Diverging->Transparent = true;
    bm41Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm41Straight, "bm41Straight", RESOURCE_PREFIX);
    bm41Straight->Transparent = true;
    bm41Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm42, "bm42", RESOURCE_PREFIX);
    bm42->Transparent = true;
    bm42->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm42Diverging, "bm42Diverging", RESOURCE_PREFIX);
    bm42Diverging->Transparent = true;
    bm42Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm42Straight, "bm42Straight", RESOURCE_PREFIX);
    bm42Straight->Transparent = true;
    bm42Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm43, "bm43", RESOURCE_PREFIX);
    bm43->Transparent = true;
    bm43->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm43Diverging, "bm43Diverging", RESOURCE_PREFIX);
    bm43Diverging->Transparent = true;
    bm43Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm43Straight, "bm43Straight", RESOURCE_PREFIX);
    bm43Straight->Transparent = true;
    bm43Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm45, "bm45", RESOURCE_PREFIX);
    bm45->Transparent = true;
    bm45->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm46, "bm46", RESOURCE_PREFIX);
    bm46->Transparent = true;
    bm46->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm50, "bm50", RESOURCE_PREFIX);
    bm50->Transparent = true;
    bm50->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm51, "bm51", RESOURCE_PREFIX);
    bm51->Transparent = true;
    bm51->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm53, "bm53", RESOURCE_PREFIX);
    bm53->Transparent = true;
    bm53->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm54, "bm54", RESOURCE_PREFIX);
    bm54->Transparent = true;
    bm54->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm56, "bm56", RESOURCE_PREFIX);
    bm56->Transparent = true;
    bm56->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm59, "bm59", RESOURCE_PREFIX);
    bm59->Transparent = true;
    bm59->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm65, "bm65", RESOURCE_PREFIX);
    bm65->Transparent = true;
    bm65->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm68CallingOn, "bm68CallingOn", RESOURCE_PREFIX);
    bm68CallingOn->Transparent = true;
    bm68CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm68dblyellow, "bm68dblyellow", RESOURCE_PREFIX);
    bm68dblyellow->Transparent = true;
    bm68dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm68grounddblred, "bm68grounddblred", RESOURCE_PREFIX);
    bm68grounddblred->Transparent = true;
    bm68grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm68grounddblwhite, "bm68grounddblwhite", RESOURCE_PREFIX);
    bm68grounddblwhite->Transparent = true;
    bm68grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm68green, "bm68green", RESOURCE_PREFIX);
    bm68green->Transparent = true;
    bm68green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm68yellow, "bm68yellow", RESOURCE_PREFIX);
    bm68yellow->Transparent = true;
    bm68yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm69CallingOn, "bm69CallingOn", RESOURCE_PREFIX);
    bm69CallingOn->Transparent = true;
    bm69CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm69dblyellow, "bm69dblyellow", RESOURCE_PREFIX);
    bm69dblyellow->Transparent = true;
    bm69dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm69grounddblred, "bm69grounddblred", RESOURCE_PREFIX);
    bm69grounddblred->Transparent = true;
    bm69grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm69grounddblwhite, "bm69grounddblwhite", RESOURCE_PREFIX);
    bm69grounddblwhite->Transparent = true;
    bm69grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm69green, "bm69green", RESOURCE_PREFIX);
    bm69green->Transparent = true;
    bm69green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm69yellow, "bm69yellow", RESOURCE_PREFIX);
    bm69yellow->Transparent = true;
    bm69yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm7, "bm7", RESOURCE_PREFIX);
    bm7->Transparent = true;
    bm7->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm70CallingOn, "bm70CallingOn", RESOURCE_PREFIX);
    bm70CallingOn->Transparent = true;
    bm70CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm70dblyellow, "bm70dblyellow", RESOURCE_PREFIX);
    bm70dblyellow->Transparent = true;
    bm70dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm70grounddblred, "bm70grounddblred", RESOURCE_PREFIX);
    bm70grounddblred->Transparent = true;
    bm70grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm70grounddblwhite, "bm70grounddblwhite", RESOURCE_PREFIX);
    bm70grounddblwhite->Transparent = true;
    bm70grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm70green, "bm70green", RESOURCE_PREFIX);
    bm70green->Transparent = true;
    bm70green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm70yellow, "bm70yellow", RESOURCE_PREFIX);
    bm70yellow->Transparent = true;
    bm70yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm71CallingOn, "bm71CallingOn", RESOURCE_PREFIX);
    bm71CallingOn->Transparent = true;
    bm71CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm71dblyellow, "bm71dblyellow", RESOURCE_PREFIX);
    bm71dblyellow->Transparent = true;
    bm71dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm71grounddblred, "bm71grounddblred", RESOURCE_PREFIX);
    bm71grounddblred->Transparent = true;
    bm71grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm71grounddblwhite, "bm71grounddblwhite", RESOURCE_PREFIX);
    bm71grounddblwhite->Transparent = true;
    bm71grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm71green, "bm71green", RESOURCE_PREFIX);
    bm71green->Transparent = true;
    bm71green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm71yellow, "bm71yellow", RESOURCE_PREFIX);
    bm71yellow->Transparent = true;
    bm71yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm72CallingOn, "bm72CallingOn", RESOURCE_PREFIX);
    bm72CallingOn->Transparent = true;
    bm72CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm72dblyellow, "bm72dblyellow", RESOURCE_PREFIX);
    bm72dblyellow->Transparent = true;
    bm72dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm72grounddblred, "bm72grounddblred", RESOURCE_PREFIX);
    bm72grounddblred->Transparent = true;
    bm72grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm72grounddblwhite, "bm72grounddblwhite", RESOURCE_PREFIX);
    bm72grounddblwhite->Transparent = true;
    bm72grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm72green, "bm72green", RESOURCE_PREFIX);
    bm72green->Transparent = true;
    bm72green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm72yellow, "bm72yellow", RESOURCE_PREFIX);
    bm72yellow->Transparent = true;
    bm72yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm73, "bm73", RESOURCE_PREFIX);
    bm73->Transparent = true;
    bm73->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm73CallingOn, "bm73CallingOn", RESOURCE_PREFIX);
    bm73CallingOn->Transparent = true;
    bm73CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm73dblyellow, "bm73dblyellow", RESOURCE_PREFIX);
    bm73dblyellow->Transparent = true;
    bm73dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm73grounddblred, "bm73grounddblred", RESOURCE_PREFIX);
    bm73grounddblred->Transparent = true;
    bm73grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm73grounddblwhite, "bm73grounddblwhite", RESOURCE_PREFIX);
    bm73grounddblwhite->Transparent = true;
    bm73grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm73green, "bm73green", RESOURCE_PREFIX);
    bm73green->Transparent = true;
    bm73green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm73yellow, "bm73yellow", RESOURCE_PREFIX);
    bm73yellow->Transparent = true;
    bm73yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm74, "bm74", RESOURCE_PREFIX);
    bm74->Transparent = true;
    bm74->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm74CallingOn, "bm74CallingOn", RESOURCE_PREFIX);
    bm74CallingOn->Transparent = true;
    bm74CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm74dblyellow, "bm74dblyellow", RESOURCE_PREFIX);
    bm74dblyellow->Transparent = true;
    bm74dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm74grounddblred, "bm74grounddblred", RESOURCE_PREFIX);
    bm74grounddblred->Transparent = true;
    bm74grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm74grounddblwhite, "bm74grounddblwhite", RESOURCE_PREFIX);
    bm74grounddblwhite->Transparent = true;
    bm74grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm74green, "bm74green", RESOURCE_PREFIX);
    bm74green->Transparent = true;
    bm74green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm74yellow, "bm74yellow", RESOURCE_PREFIX);
    bm74yellow->Transparent = true;
    bm74yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm75CallingOn, "bm75CallingOn", RESOURCE_PREFIX);
    bm75CallingOn->Transparent = true;
    bm75CallingOn->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm75dblyellow, "bm75dblyellow", RESOURCE_PREFIX);
    bm75dblyellow->Transparent = true;
    bm75dblyellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm75grounddblred, "bm75grounddblred", RESOURCE_PREFIX);
    bm75grounddblred->Transparent = true;
    bm75grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm75grounddblwhite, "bm75grounddblwhite", RESOURCE_PREFIX);
    bm75grounddblwhite->Transparent = true;
    bm75grounddblwhite->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm75green, "bm75green", RESOURCE_PREFIX);
    bm75green->Transparent = true;
    bm75green->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm75yellow, "bm75yellow", RESOURCE_PREFIX);
    bm75yellow->Transparent = true;
    bm75yellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm77, "bm77", RESOURCE_PREFIX);
    bm77->Transparent = true;
    bm77->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm77Striped, "bm77Striped", RESOURCE_PREFIX);
    bm77Striped->Transparent = true;
    bm77Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm78, "bm78", RESOURCE_PREFIX);
    bm78->Transparent = true;
    bm78->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm78Striped, "bm78Striped", RESOURCE_PREFIX);
    bm78Striped->Transparent = true;
    bm78Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm7Diverging, "bm7Diverging", RESOURCE_PREFIX);
    bm7Diverging->Transparent = true;
    bm7Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm7Straight, "bm7Straight", RESOURCE_PREFIX);
    bm7Straight->Transparent = true;
    bm7Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm8, "bm8", RESOURCE_PREFIX);
    bm8->Transparent = true;
    bm8->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm85, "bm85", RESOURCE_PREFIX);
    bm85->Transparent = true;
    bm85->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm8Diverging, "bm8Diverging", RESOURCE_PREFIX);
    bm8Diverging->Transparent = true;
    bm8Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm8Straight, "bm8Straight", RESOURCE_PREFIX);
    bm8Straight->Transparent = true;
    bm8Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm9, "bm9", RESOURCE_PREFIX);
    bm9->Transparent = true;
    bm9->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm93set, "bm93set", RESOURCE_PREFIX);
    bm93set->Transparent = true;
    bm93set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm93unset, "bm93unset", RESOURCE_PREFIX);
    bm93unset->Transparent = true;
    bm93unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm94set, "bm94set", RESOURCE_PREFIX);
    bm94set->Transparent = true;
    bm94set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm94unset, "bm94unset", RESOURCE_PREFIX);
    bm94unset->Transparent = true;
    bm94unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm9Diverging, "bm9Diverging", RESOURCE_PREFIX);
    bm9Diverging->Transparent = true;
    bm9Diverging->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bm9Straight, "bm9Straight", RESOURCE_PREFIX);
    bm9Straight->Transparent = true;
    bm9Straight->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmGreenEllipse, "bmGreenEllipse", RESOURCE_PREFIX);
    bmGreenEllipse->Transparent = true;
    bmGreenEllipse->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmGreenRect, "bmGreenRect", RESOURCE_PREFIX);
    bmGreenRect->Transparent = true;
    bmGreenRect->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmGrid, "bmGrid", RESOURCE_PREFIX);
    bmGrid->Transparent = true;
    bmGrid->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmLightBlueRect, "bmLightBlueRect", RESOURCE_PREFIX);
    bmLightBlueRect->Transparent = true;
    bmLightBlueRect->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmName, "bmName", RESOURCE_PREFIX);
    bmName->Transparent = true;
    bmName->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmNameStriped, "bmNameStriped", RESOURCE_PREFIX);
    bmNameStriped->Transparent = true;
    bmNameStriped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRedEllipse, "bmRedEllipse", RESOURCE_PREFIX);
    bmRedEllipse->Transparent = true;
    bmRedEllipse->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRedRect, "bmRedRect", RESOURCE_PREFIX);
    bmRedRect->Transparent = true;
    bmRedRect->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink1, "bmRtCancELink1", RESOURCE_PREFIX);
    bmRtCancELink1->Transparent = true;
    bmRtCancELink1->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink2, "bmRtCancELink2", RESOURCE_PREFIX);
    bmRtCancELink2->Transparent = true;
    bmRtCancELink2->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink3, "bmRtCancELink3", RESOURCE_PREFIX);
    bmRtCancELink3->Transparent = true;
    bmRtCancELink3->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink4, "bmRtCancELink4", RESOURCE_PREFIX);
    bmRtCancELink4->Transparent = true;
    bmRtCancELink4->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink6, "bmRtCancELink6", RESOURCE_PREFIX);
    bmRtCancELink6->Transparent = true;
    bmRtCancELink6->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink7, "bmRtCancELink7", RESOURCE_PREFIX);
    bmRtCancELink7->Transparent = true;
    bmRtCancELink7->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink8, "bmRtCancELink8", RESOURCE_PREFIX);
    bmRtCancELink8->Transparent = true;
    bmRtCancELink8->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(bmRtCancELink9, "bmRtCancELink9", RESOURCE_PREFIX);
    bmRtCancELink9->Transparent = true;
    bmRtCancELink9->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br1, "br1", RESOURCE_PREFIX);
    br1->Transparent = true;
    br1->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br10, "br10", RESOURCE_PREFIX);
    br10->Transparent = true;
    br10->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br11, "br11", RESOURCE_PREFIX);
    br11->Transparent = true;
    br11->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br12, "br12", RESOURCE_PREFIX);
    br12->Transparent = true;
    br12->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br2, "br2", RESOURCE_PREFIX);
    br2->Transparent = true;
    br2->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br3, "br3", RESOURCE_PREFIX);
    br3->Transparent = true;
    br3->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br4, "br4", RESOURCE_PREFIX);
    br4->Transparent = true;
    br4->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br5, "br5", RESOURCE_PREFIX);
    br5->Transparent = true;
    br5->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br6, "br6", RESOURCE_PREFIX);
    br6->Transparent = true;
    br6->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br7, "br7", RESOURCE_PREFIX);
    br7->Transparent = true;
    br7->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br8, "br8", RESOURCE_PREFIX);
    br8->Transparent = true;
    br8->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(br9, "br9", RESOURCE_PREFIX);
    br9->Transparent = true;
    br9->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Concourse, "Concourse", RESOURCE_PREFIX);
    Concourse->Transparent = true;
    Concourse->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ConcourseGlyph, "ConcourseGlyph", RESOURCE_PREFIX);
    ConcourseGlyph->Transparent = true;
    ConcourseGlyph->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ConcourseStriped, "ConcourseStriped", RESOURCE_PREFIX);
    ConcourseStriped->Transparent = true;
    ConcourseStriped->TransparentColor = clB5G5R5;

    loadResourceFromPrefix(CouplingExit1, "CouplingExit1", RESOURCE_PREFIX);
    CouplingExit1->Transparent = true;
    CouplingExit1->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(CouplingExit2, "CouplingExit2", RESOURCE_PREFIX);
    CouplingExit2->Transparent = true;
    CouplingExit2->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(CouplingExit3, "CouplingExit3", RESOURCE_PREFIX);
    CouplingExit3->Transparent = true;
    CouplingExit3->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(CouplingExit4, "CouplingExit4", RESOURCE_PREFIX);
    CouplingExit4->Transparent = true;
    CouplingExit4->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(CouplingExit6, "CouplingExit6", RESOURCE_PREFIX);
    CouplingExit6->Transparent = true;
    CouplingExit6->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(CouplingExit7, "CouplingExit7", RESOURCE_PREFIX);
    CouplingExit7->Transparent = true;
    CouplingExit7->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(CouplingExit8, "CouplingExit8", RESOURCE_PREFIX);
    CouplingExit8->Transparent = true;
    CouplingExit8->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(CouplingExit9, "CouplingExit9", RESOURCE_PREFIX);
    CouplingExit9->Transparent = true;
    CouplingExit9->TransparentColor = clB5G5R5;

    loadResourceFromPrefix(SolidCircleRed, "SolidCircleRed", RESOURCE_PREFIX);
    SolidCircleRed->Transparent = true;
    SolidCircleRed->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(SolidCircleYellow, "SolidCircleYellow", RESOURCE_PREFIX);
    SolidCircleYellow->Transparent = true;
    SolidCircleYellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(SolidCircleGreen, "SolidCircleGreen", RESOURCE_PREFIX);
    SolidCircleGreen->Transparent = true;
    SolidCircleGreen->TransparentColor = clB5G5R5;

    loadResourceFromPrefix(ELk1, "ELk1", RESOURCE_PREFIX);
    ELk1->Transparent = true;
    ELk1->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ELk2, "ELk2", RESOURCE_PREFIX);
    ELk2->Transparent = true;
    ELk2->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ELk3, "ELk3", RESOURCE_PREFIX);
    ELk3->Transparent = true;
    ELk3->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ELk4, "ELk4", RESOURCE_PREFIX);
    ELk4->Transparent = true;
    ELk4->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ELk6, "ELk6", RESOURCE_PREFIX);
    ELk6->Transparent = true;
    ELk6->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ELk7, "ELk7", RESOURCE_PREFIX);
    ELk7->Transparent = true;
    ELk7->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ELk8, "ELk8", RESOURCE_PREFIX);
    ELk8->Transparent = true;
    ELk8->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(ELk9, "ELk9", RESOURCE_PREFIX);
    ELk9->Transparent = true;
    ELk9->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(BlackOctagon, "BlackOctagon", RESOURCE_PREFIX);
    BlackOctagon->Transparent = true;
    BlackOctagon->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl1, "gl1", RESOURCE_PREFIX);
    gl1->Transparent = true;
    gl1->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl10, "gl10", RESOURCE_PREFIX);
    gl10->Transparent = true;
    gl10->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl100, "gl100", RESOURCE_PREFIX);
    gl100->Transparent = true;
    gl100->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl101, "gl101", RESOURCE_PREFIX);
    gl101->Transparent = true;
    gl101->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl102, "gl102", RESOURCE_PREFIX);
    gl102->Transparent = true;
    gl102->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl103, "gl103", RESOURCE_PREFIX);
    gl103->Transparent = true;
    gl103->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl104, "gl104", RESOURCE_PREFIX);
    gl104->Transparent = true;
    gl104->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl105, "gl105", RESOURCE_PREFIX);
    gl105->Transparent = true;
    gl105->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl106, "gl106", RESOURCE_PREFIX);
    gl106->Transparent = true;
    gl106->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl107, "gl107", RESOURCE_PREFIX);
    gl107->Transparent = true;
    gl107->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl108, "gl108", RESOURCE_PREFIX);
    gl108->Transparent = true;
    gl108->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl109, "gl109", RESOURCE_PREFIX);
    gl109->Transparent = true;
    gl109->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl11, "gl11", RESOURCE_PREFIX);
    gl11->Transparent = true;
    gl11->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl110, "gl110", RESOURCE_PREFIX);
    gl110->Transparent = true;
    gl110->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl111, "gl111", RESOURCE_PREFIX);
    gl111->Transparent = true;
    gl111->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl112, "gl112", RESOURCE_PREFIX);
    gl112->Transparent = true;
    gl112->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl113, "gl113", RESOURCE_PREFIX);
    gl113->Transparent = true;
    gl113->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl114, "gl114", RESOURCE_PREFIX);
    gl114->Transparent = true;
    gl114->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl115, "gl115", RESOURCE_PREFIX);
    gl115->Transparent = true;
    gl115->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl116, "gl116", RESOURCE_PREFIX);
    gl116->Transparent = true;
    gl116->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl117, "gl117", RESOURCE_PREFIX);
    gl117->Transparent = true;
    gl117->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl118, "gl118", RESOURCE_PREFIX);
    gl118->Transparent = true;
    gl118->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl119, "gl119", RESOURCE_PREFIX);
    gl119->Transparent = true;
    gl119->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl12, "gl12", RESOURCE_PREFIX);
    gl12->Transparent = true;
    gl12->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl120, "gl120", RESOURCE_PREFIX);
    gl120->Transparent = true;
    gl120->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl121, "gl121", RESOURCE_PREFIX);
    gl121->Transparent = true;
    gl121->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl122, "gl122", RESOURCE_PREFIX);
    gl122->Transparent = true;
    gl122->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl123, "gl123", RESOURCE_PREFIX);
    gl123->Transparent = true;
    gl123->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl124, "gl124", RESOURCE_PREFIX);
    gl124->Transparent = true;
    gl124->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl125, "gl125", RESOURCE_PREFIX);
    gl125->Transparent = true;
    gl125->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl126, "gl126", RESOURCE_PREFIX);
    gl126->Transparent = true;
    gl126->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl127, "gl127", RESOURCE_PREFIX);
    gl127->Transparent = true;
    gl127->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl128, "gl128", RESOURCE_PREFIX);
    gl128->Transparent = true;
    gl128->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl129, "gl129", RESOURCE_PREFIX);
    gl129->Transparent = true;
    gl129->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl129Striped, "gl129Striped", RESOURCE_PREFIX);
    gl129Striped->Transparent = true;
    gl129Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl13, "gl13", RESOURCE_PREFIX);
    gl13->Transparent = true;
    gl13->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl130, "gl130", RESOURCE_PREFIX);
    gl130->Transparent = true;
    gl130->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl130Striped, "gl130Striped", RESOURCE_PREFIX);
    gl130Striped->Transparent = true;
    gl130Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl131, "gl131", RESOURCE_PREFIX);
    gl131->Transparent = true;
    gl131->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl132, "gl132", RESOURCE_PREFIX);
    gl132->Transparent = true;
    gl132->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl133, "gl133", RESOURCE_PREFIX);
    gl133->Transparent = true;
    gl133->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl134, "gl134", RESOURCE_PREFIX);
    gl134->Transparent = true;
    gl134->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl135, "gl135", RESOURCE_PREFIX);
    gl135->Transparent = true;
    gl135->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl136, "gl136", RESOURCE_PREFIX);
    gl136->Transparent = true;
    gl136->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl137, "gl137", RESOURCE_PREFIX);
    gl137->Transparent = true;
    gl137->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl138, "gl138", RESOURCE_PREFIX);
    gl138->Transparent = true;
    gl138->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl139, "gl139", RESOURCE_PREFIX);
    gl139->Transparent = true;
    gl139->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl14, "gl14", RESOURCE_PREFIX);
    gl14->Transparent = true;
    gl14->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl140, "gl140", RESOURCE_PREFIX);
    gl140->Transparent = true;
    gl140->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl141, "gl141", RESOURCE_PREFIX);
    gl141->Transparent = true;
    gl141->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl142, "gl142", RESOURCE_PREFIX);
    gl142->Transparent = true;
    gl142->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl143, "gl143", RESOURCE_PREFIX);
    gl143->Transparent = true;
    gl143->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl145, "gl145", RESOURCE_PREFIX);
    gl145->Transparent = true;
    gl145->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl145Striped, "gl145Striped", RESOURCE_PREFIX);
    gl145Striped->Transparent = true;
    gl145Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl146, "gl146", RESOURCE_PREFIX);
    gl146->Transparent = true;
    gl146->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl146Striped, "gl146Striped", RESOURCE_PREFIX);
    gl146Striped->Transparent = true;
    gl146Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl15, "gl15", RESOURCE_PREFIX);
    gl15->Transparent = true;
    gl15->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl16, "gl16", RESOURCE_PREFIX);
    gl16->Transparent = true;
    gl16->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl18, "gl18", RESOURCE_PREFIX);
    gl18->Transparent = true;
    gl18->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl19, "gl19", RESOURCE_PREFIX);
    gl19->Transparent = true;
    gl19->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl2, "gl2", RESOURCE_PREFIX);
    gl2->Transparent = true;
    gl2->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl20, "gl20", RESOURCE_PREFIX);
    gl20->Transparent = true;
    gl20->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl21, "gl21", RESOURCE_PREFIX);
    gl21->Transparent = true;
    gl21->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl22, "gl22", RESOURCE_PREFIX);
    gl22->Transparent = true;
    gl22->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl23, "gl23", RESOURCE_PREFIX);
    gl23->Transparent = true;
    gl23->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl24, "gl24", RESOURCE_PREFIX);
    gl24->Transparent = true;
    gl24->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl25, "gl25", RESOURCE_PREFIX);
    gl25->Transparent = true;
    gl25->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl26, "gl26", RESOURCE_PREFIX);
    gl26->Transparent = true;
    gl26->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl27, "gl27", RESOURCE_PREFIX);
    gl27->Transparent = true;
    gl27->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl28, "gl28", RESOURCE_PREFIX);
    gl28->Transparent = true;
    gl28->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl29, "gl29", RESOURCE_PREFIX);
    gl29->Transparent = true;
    gl29->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl3, "gl3", RESOURCE_PREFIX);
    gl3->Transparent = true;
    gl3->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl30, "gl30", RESOURCE_PREFIX);
    gl30->Transparent = true;
    gl30->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl31, "gl31", RESOURCE_PREFIX);
    gl31->Transparent = true;
    gl31->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl32, "gl32", RESOURCE_PREFIX);
    gl32->Transparent = true;
    gl32->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl33, "gl33", RESOURCE_PREFIX);
    gl33->Transparent = true;
    gl33->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl34, "gl34", RESOURCE_PREFIX);
    gl34->Transparent = true;
    gl34->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl35, "gl35", RESOURCE_PREFIX);
    gl35->Transparent = true;
    gl35->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl36, "gl36", RESOURCE_PREFIX);
    gl36->Transparent = true;
    gl36->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl37, "gl37", RESOURCE_PREFIX);
    gl37->Transparent = true;
    gl37->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl38, "gl38", RESOURCE_PREFIX);
    gl38->Transparent = true;
    gl38->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl39, "gl39", RESOURCE_PREFIX);
    gl39->Transparent = true;
    gl39->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl4, "gl4", RESOURCE_PREFIX);
    gl4->Transparent = true;
    gl4->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl40, "gl40", RESOURCE_PREFIX);
    gl40->Transparent = true;
    gl40->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl41, "gl41", RESOURCE_PREFIX);
    gl41->Transparent = true;
    gl41->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl42, "gl42", RESOURCE_PREFIX);
    gl42->Transparent = true;
    gl42->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl43, "gl43", RESOURCE_PREFIX);
    gl43->Transparent = true;
    gl43->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl44, "gl44", RESOURCE_PREFIX);
    gl44->Transparent = true;
    gl44->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl45, "gl45", RESOURCE_PREFIX);
    gl45->Transparent = true;
    gl45->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl46, "gl46", RESOURCE_PREFIX);
    gl46->Transparent = true;
    gl46->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl47, "gl47", RESOURCE_PREFIX);
    gl47->Transparent = true;
    gl47->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl48, "gl48", RESOURCE_PREFIX);
    gl48->Transparent = true;
    gl48->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl49, "gl49", RESOURCE_PREFIX);
    gl49->Transparent = true;
    gl49->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl5, "gl5", RESOURCE_PREFIX);
    gl5->Transparent = true;
    gl5->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl50, "gl50", RESOURCE_PREFIX);
    gl50->Transparent = true;
    gl50->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl51, "gl51", RESOURCE_PREFIX);
    gl51->Transparent = true;
    gl51->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl52, "gl52", RESOURCE_PREFIX);
    gl52->Transparent = true;
    gl52->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl53, "gl53", RESOURCE_PREFIX);
    gl53->Transparent = true;
    gl53->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl54, "gl54", RESOURCE_PREFIX);
    gl54->Transparent = true;
    gl54->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl55, "gl55", RESOURCE_PREFIX);
    gl55->Transparent = true;
    gl55->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl56, "gl56", RESOURCE_PREFIX);
    gl56->Transparent = true;
    gl56->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl57, "gl57", RESOURCE_PREFIX);
    gl57->Transparent = true;
    gl57->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl58, "gl58", RESOURCE_PREFIX);
    gl58->Transparent = true;
    gl58->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl59, "gl59", RESOURCE_PREFIX);
    gl59->Transparent = true;
    gl59->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl6, "gl6", RESOURCE_PREFIX);
    gl6->Transparent = true;
    gl6->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl60, "gl60", RESOURCE_PREFIX);
    gl60->Transparent = true;
    gl60->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl61, "gl61", RESOURCE_PREFIX);
    gl61->Transparent = true;
    gl61->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl62, "gl62", RESOURCE_PREFIX);
    gl62->Transparent = true;
    gl62->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl63, "gl63", RESOURCE_PREFIX);
    gl63->Transparent = true;
    gl63->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl64, "gl64", RESOURCE_PREFIX);
    gl64->Transparent = true;
    gl64->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl65, "gl65", RESOURCE_PREFIX);
    gl65->Transparent = true;
    gl65->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl66, "gl66", RESOURCE_PREFIX);
    gl66->Transparent = true;
    gl66->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl67, "gl67", RESOURCE_PREFIX);
    gl67->Transparent = true;
    gl67->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl68, "gl68", RESOURCE_PREFIX);
    gl68->Transparent = true;
    gl68->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl69, "gl69", RESOURCE_PREFIX);
    gl69->Transparent = true;
    gl69->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl7, "gl7", RESOURCE_PREFIX);
    gl7->Transparent = true;
    gl7->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl70, "gl70", RESOURCE_PREFIX);
    gl70->Transparent = true;
    gl70->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl71, "gl71", RESOURCE_PREFIX);
    gl71->Transparent = true;
    gl71->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl72, "gl72", RESOURCE_PREFIX);
    gl72->Transparent = true;
    gl72->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl73, "gl73", RESOURCE_PREFIX);
    gl73->Transparent = true;
    gl73->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl73grounddblred, "gl73grounddblred", RESOURCE_PREFIX);
    gl73grounddblred->Transparent = true;
    gl73grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl74, "gl74", RESOURCE_PREFIX);
    gl74->Transparent = true;
    gl74->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl74grounddblred, "gl74grounddblred", RESOURCE_PREFIX);
    gl74grounddblred->Transparent = true;
    gl74grounddblred->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl75, "gl75", RESOURCE_PREFIX);
    gl75->Transparent = true;
    gl75->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl76, "gl76", RESOURCE_PREFIX);
    gl76->Transparent = true;
    gl76->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl76Striped, "gl76Striped", RESOURCE_PREFIX);
    gl76Striped->Transparent = true;
    gl76Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl77, "gl77", RESOURCE_PREFIX);
    gl77->Transparent = true;
    gl77->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl78, "gl78", RESOURCE_PREFIX);
    gl78->Transparent = true;
    gl78->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl79, "gl79", RESOURCE_PREFIX);
    gl79->Transparent = true;
    gl79->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl79Striped, "gl79Striped", RESOURCE_PREFIX);
    gl79Striped->Transparent = true;
    gl79Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl8, "gl8", RESOURCE_PREFIX);
    gl8->Transparent = true;
    gl8->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl80, "gl80", RESOURCE_PREFIX);
    gl80->Transparent = true;
    gl80->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl81, "gl81", RESOURCE_PREFIX);
    gl81->Transparent = true;
    gl81->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl82, "gl82", RESOURCE_PREFIX);
    gl82->Transparent = true;
    gl82->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl83, "gl83", RESOURCE_PREFIX);
    gl83->Transparent = true;
    gl83->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl84, "gl84", RESOURCE_PREFIX);
    gl84->Transparent = true;
    gl84->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl85, "gl85", RESOURCE_PREFIX);
    gl85->Transparent = true;
    gl85->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl86, "gl86", RESOURCE_PREFIX);
    gl86->Transparent = true;
    gl86->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl87, "gl87", RESOURCE_PREFIX);
    gl87->Transparent = true;
    gl87->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl88set, "gl88set", RESOURCE_PREFIX);
    gl88set->Transparent = true;
    gl88set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl88unset, "gl88unset", RESOURCE_PREFIX);
    gl88unset->Transparent = true;
    gl88unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl89set, "gl89set", RESOURCE_PREFIX);
    gl89set->Transparent = true;
    gl89set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl89unset, "gl89unset", RESOURCE_PREFIX);
    gl89unset->Transparent = true;
    gl89unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl9, "gl9", RESOURCE_PREFIX);
    gl9->Transparent = true;
    gl9->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl90set, "gl90set", RESOURCE_PREFIX);
    gl90set->Transparent = true;
    gl90set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl90unset, "gl90unset", RESOURCE_PREFIX);
    gl90unset->Transparent = true;
    gl90unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl91set, "gl91set", RESOURCE_PREFIX);
    gl91set->Transparent = true;
    gl91set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl91unset, "gl91unset", RESOURCE_PREFIX);
    gl91unset->Transparent = true;
    gl91unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl92set, "gl92set", RESOURCE_PREFIX);
    gl92set->Transparent = true;
    gl92set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl92unset, "gl92unset", RESOURCE_PREFIX);
    gl92unset->Transparent = true;
    gl92unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl93set, "gl93set", RESOURCE_PREFIX);
    gl93set->Transparent = true;
    gl93set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl94set, "gl94set", RESOURCE_PREFIX);
    gl94set->Transparent = true;
    gl94set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl95set, "gl95set", RESOURCE_PREFIX);
    gl95set->Transparent = true;
    gl95set->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl95unset, "gl95unset", RESOURCE_PREFIX);
    gl95unset->Transparent = true;
    gl95unset->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl97, "gl97", RESOURCE_PREFIX);
    gl97->Transparent = true;
    gl97->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl98, "gl98", RESOURCE_PREFIX);
    gl98->Transparent = true;
    gl98->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(gl99, "gl99", RESOURCE_PREFIX);
    gl99->Transparent = true;
    gl99->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat68, "Plat68", RESOURCE_PREFIX);
    Plat68->Transparent = true;
    Plat68->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat68Striped, "Plat68Striped", RESOURCE_PREFIX);
    Plat68Striped->Transparent = true;
    Plat68Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat69, "Plat69", RESOURCE_PREFIX);
    Plat69->Transparent = true;
    Plat69->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat69Striped, "Plat69Striped", RESOURCE_PREFIX);
    Plat69Striped->Transparent = true;
    Plat69Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat70, "Plat70", RESOURCE_PREFIX);
    Plat70->Transparent = true;
    Plat70->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat70Striped, "Plat70Striped", RESOURCE_PREFIX);
    Plat70Striped->Transparent = true;
    Plat70Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat71, "Plat71", RESOURCE_PREFIX);
    Plat71->Transparent = true;
    Plat71->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(Plat71Striped, "Plat71Striped", RESOURCE_PREFIX);
    Plat71Striped->Transparent = true;
    Plat71Striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm1, "sm1", RESOURCE_PREFIX);
    sm1->Transparent = true;
    sm1->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm10, "sm10", RESOURCE_PREFIX);
    sm10->Transparent = true;
    sm10->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm100, "sm100", RESOURCE_PREFIX);
    sm100->Transparent = true;
    sm100->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm101, "sm101", RESOURCE_PREFIX);
    sm101->Transparent = true;
    sm101->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm102, "sm102", RESOURCE_PREFIX);
    sm102->Transparent = true;
    sm102->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm103, "sm103", RESOURCE_PREFIX);
    sm103->Transparent = true;
    sm103->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm104, "sm104", RESOURCE_PREFIX);
    sm104->Transparent = true;
    sm104->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm105, "sm105", RESOURCE_PREFIX);
    sm105->Transparent = true;
    sm105->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm106, "sm106", RESOURCE_PREFIX);
    sm106->Transparent = true;
    sm106->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm107, "sm107", RESOURCE_PREFIX);
    sm107->Transparent = true;
    sm107->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm108, "sm108", RESOURCE_PREFIX);
    sm108->Transparent = true;
    sm108->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm109, "sm109", RESOURCE_PREFIX);
    sm109->Transparent = true;
    sm109->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm11, "sm11", RESOURCE_PREFIX);
    sm11->Transparent = true;
    sm11->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm110, "sm110", RESOURCE_PREFIX);
    sm110->Transparent = true;
    sm110->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm111, "sm111", RESOURCE_PREFIX);
    sm111->Transparent = true;
    sm111->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm112, "sm112", RESOURCE_PREFIX);
    sm112->Transparent = true;
    sm112->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm115, "sm115", RESOURCE_PREFIX);
    sm115->Transparent = true;
    sm115->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm117, "sm117", RESOURCE_PREFIX);
    sm117->Transparent = true;
    sm117->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm12, "sm12", RESOURCE_PREFIX);
    sm12->Transparent = true;
    sm12->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm129, "sm129", RESOURCE_PREFIX);
    sm129->Transparent = true;
    sm129->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm129striped, "sm129striped", RESOURCE_PREFIX);
    sm129striped->Transparent = true;
    sm129striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm13, "sm13", RESOURCE_PREFIX);
    sm13->Transparent = true;
    sm13->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm130, "sm130", RESOURCE_PREFIX);
    sm130->Transparent = true;
    sm130->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm130striped, "sm130striped", RESOURCE_PREFIX);
    sm130striped->Transparent = true;
    sm130striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm131striped, "sm131striped", RESOURCE_PREFIX);
    sm131striped->Transparent = true;
    sm131striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm132, "sm132", RESOURCE_PREFIX);
    sm132->Transparent = true;
    sm132->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm133, "sm133", RESOURCE_PREFIX);
    sm133->Transparent = true;
    sm133->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm134, "sm134", RESOURCE_PREFIX);
    sm134->Transparent = true;
    sm134->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm135, "sm135", RESOURCE_PREFIX);
    sm135->Transparent = true;
    sm135->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm136, "sm136", RESOURCE_PREFIX);
    sm136->Transparent = true;
    sm136->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm137, "sm137", RESOURCE_PREFIX);
    sm137->Transparent = true;
    sm137->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm138, "sm138", RESOURCE_PREFIX);
    sm138->Transparent = true;
    sm138->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm139, "sm139", RESOURCE_PREFIX);
    sm139->Transparent = true;
    sm139->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm14, "sm14", RESOURCE_PREFIX);
    sm14->Transparent = true;
    sm14->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm15, "sm15", RESOURCE_PREFIX);
    sm15->Transparent = true;
    sm15->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm16, "sm16", RESOURCE_PREFIX);
    sm16->Transparent = true;
    sm16->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm18, "sm18", RESOURCE_PREFIX);
    sm18->Transparent = true;
    sm18->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm19, "sm19", RESOURCE_PREFIX);
    sm19->Transparent = true;
    sm19->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm2, "sm2", RESOURCE_PREFIX);
    sm2->Transparent = true;
    sm2->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm20, "sm20", RESOURCE_PREFIX);
    sm20->Transparent = true;
    sm20->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm21, "sm21", RESOURCE_PREFIX);
    sm21->Transparent = true;
    sm21->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm22, "sm22", RESOURCE_PREFIX);
    sm22->Transparent = true;
    sm22->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm23, "sm23", RESOURCE_PREFIX);
    sm23->Transparent = true;
    sm23->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm24, "sm24", RESOURCE_PREFIX);
    sm24->Transparent = true;
    sm24->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm25, "sm25", RESOURCE_PREFIX);
    sm25->Transparent = true;
    sm25->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm26, "sm26", RESOURCE_PREFIX);
    sm26->Transparent = true;
    sm26->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm27, "sm27", RESOURCE_PREFIX);
    sm27->Transparent = true;
    sm27->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm28, "sm28", RESOURCE_PREFIX);
    sm28->Transparent = true;
    sm28->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm29, "sm29", RESOURCE_PREFIX);
    sm29->Transparent = true;
    sm29->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm3, "sm3", RESOURCE_PREFIX);
    sm3->Transparent = true;
    sm3->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm30, "sm30", RESOURCE_PREFIX);
    sm30->Transparent = true;
    sm30->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm31, "sm31", RESOURCE_PREFIX);
    sm31->Transparent = true;
    sm31->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm32, "sm32", RESOURCE_PREFIX);
    sm32->Transparent = true;
    sm32->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm33, "sm33", RESOURCE_PREFIX);
    sm33->Transparent = true;
    sm33->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm34, "sm34", RESOURCE_PREFIX);
    sm34->Transparent = true;
    sm34->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm35, "sm35", RESOURCE_PREFIX);
    sm35->Transparent = true;
    sm35->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm36, "sm36", RESOURCE_PREFIX);
    sm36->Transparent = true;
    sm36->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm37, "sm37", RESOURCE_PREFIX);
    sm37->Transparent = true;
    sm37->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm38, "sm38", RESOURCE_PREFIX);
    sm38->Transparent = true;
    sm38->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm39, "sm39", RESOURCE_PREFIX);
    sm39->Transparent = true;
    sm39->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm4, "sm4", RESOURCE_PREFIX);
    sm4->Transparent = true;
    sm4->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm40, "sm40", RESOURCE_PREFIX);
    sm40->Transparent = true;
    sm40->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm41, "sm41", RESOURCE_PREFIX);
    sm41->Transparent = true;
    sm41->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm42, "sm42", RESOURCE_PREFIX);
    sm42->Transparent = true;
    sm42->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm43, "sm43", RESOURCE_PREFIX);
    sm43->Transparent = true;
    sm43->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm44, "sm44", RESOURCE_PREFIX);
    sm44->Transparent = true;
    sm44->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm45, "sm45", RESOURCE_PREFIX);
    sm45->Transparent = true;
    sm45->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm46, "sm46", RESOURCE_PREFIX);
    sm46->Transparent = true;
    sm46->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm47, "sm47", RESOURCE_PREFIX);
    sm47->Transparent = true;
    sm47->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm48, "sm48", RESOURCE_PREFIX);
    sm48->Transparent = true;
    sm48->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm49, "sm49", RESOURCE_PREFIX);
    sm49->Transparent = true;
    sm49->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm5, "sm5", RESOURCE_PREFIX);
    sm5->Transparent = true;
    sm5->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm50, "sm50", RESOURCE_PREFIX);
    sm50->Transparent = true;
    sm50->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm51, "sm51", RESOURCE_PREFIX);
    sm51->Transparent = true;
    sm51->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm52, "sm52", RESOURCE_PREFIX);
    sm52->Transparent = true;
    sm52->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm53, "sm53", RESOURCE_PREFIX);
    sm53->Transparent = true;
    sm53->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm54, "sm54", RESOURCE_PREFIX);
    sm54->Transparent = true;
    sm54->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm55, "sm55", RESOURCE_PREFIX);
    sm55->Transparent = true;
    sm55->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm56, "sm56", RESOURCE_PREFIX);
    sm56->Transparent = true;
    sm56->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm57, "sm57", RESOURCE_PREFIX);
    sm57->Transparent = true;
    sm57->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm58, "sm58", RESOURCE_PREFIX);
    sm58->Transparent = true;
    sm58->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm59, "sm59", RESOURCE_PREFIX);
    sm59->Transparent = true;
    sm59->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm6, "sm6", RESOURCE_PREFIX);
    sm6->Transparent = true;
    sm6->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm60, "sm60", RESOURCE_PREFIX);
    sm60->Transparent = true;
    sm60->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm61, "sm61", RESOURCE_PREFIX);
    sm61->Transparent = true;
    sm61->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm62, "sm62", RESOURCE_PREFIX);
    sm62->Transparent = true;
    sm62->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm63, "sm63", RESOURCE_PREFIX);
    sm63->Transparent = true;
    sm63->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm64, "sm64", RESOURCE_PREFIX);
    sm64->Transparent = true;
    sm64->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm65, "sm65", RESOURCE_PREFIX);
    sm65->Transparent = true;
    sm65->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm66, "sm66", RESOURCE_PREFIX);
    sm66->Transparent = true;
    sm66->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm67, "sm67", RESOURCE_PREFIX);
    sm67->Transparent = true;
    sm67->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm7, "sm7", RESOURCE_PREFIX);
    sm7->Transparent = true;
    sm7->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm76, "sm76", RESOURCE_PREFIX);
    sm76->Transparent = true;
    sm76->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm76striped, "sm76striped", RESOURCE_PREFIX);
    sm76striped->Transparent = true;
    sm76striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm77, "sm77", RESOURCE_PREFIX);
    sm77->Transparent = true;
    sm77->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm77striped, "sm77striped", RESOURCE_PREFIX);
    sm77striped->Transparent = true;
    sm77striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm78, "sm78", RESOURCE_PREFIX);
    sm78->Transparent = true;
    sm78->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm78striped, "sm78striped", RESOURCE_PREFIX);
    sm78striped->Transparent = true;
    sm78striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm79, "sm79", RESOURCE_PREFIX);
    sm79->Transparent = true;
    sm79->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm79striped, "sm79striped", RESOURCE_PREFIX);
    sm79striped->Transparent = true;
    sm79striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm8, "sm8", RESOURCE_PREFIX);
    sm8->Transparent = true;
    sm8->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm80, "sm80", RESOURCE_PREFIX);
    sm80->Transparent = true;
    sm80->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm81, "sm81", RESOURCE_PREFIX);
    sm81->Transparent = true;
    sm81->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm82, "sm82", RESOURCE_PREFIX);
    sm82->Transparent = true;
    sm82->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm83, "sm83", RESOURCE_PREFIX);
    sm83->Transparent = true;
    sm83->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm84, "sm84", RESOURCE_PREFIX);
    sm84->Transparent = true;
    sm84->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm85, "sm85", RESOURCE_PREFIX);
    sm85->Transparent = true;
    sm85->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm86, "sm86", RESOURCE_PREFIX);
    sm86->Transparent = true;
    sm86->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm87, "sm87", RESOURCE_PREFIX);
    sm87->Transparent = true;
    sm87->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm88, "sm88", RESOURCE_PREFIX);
    sm88->Transparent = true;
    sm88->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm89, "sm89", RESOURCE_PREFIX);
    sm89->Transparent = true;
    sm89->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm9, "sm9", RESOURCE_PREFIX);
    sm9->Transparent = true;
    sm9->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm90, "sm90", RESOURCE_PREFIX);
    sm90->Transparent = true;
    sm90->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm91, "sm91", RESOURCE_PREFIX);
    sm91->Transparent = true;
    sm91->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm92, "sm92", RESOURCE_PREFIX);
    sm92->Transparent = true;
    sm92->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm93, "sm93", RESOURCE_PREFIX);
    sm93->Transparent = true;
    sm93->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm94, "sm94", RESOURCE_PREFIX);
    sm94->Transparent = true;
    sm94->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm95, "sm95", RESOURCE_PREFIX);
    sm95->Transparent = true;
    sm95->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm96, "sm96", RESOURCE_PREFIX);
    sm96->Transparent = true;
    sm96->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm96striped, "sm96striped", RESOURCE_PREFIX);
    sm96striped->Transparent = true;
    sm96striped->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm97, "sm97", RESOURCE_PREFIX);
    sm97->Transparent = true;
    sm97->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm98, "sm98", RESOURCE_PREFIX);
    sm98->Transparent = true;
    sm98->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(sm99, "sm99", RESOURCE_PREFIX);
    sm99->Transparent = true;
    sm99->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smBlack, "smBlack", RESOURCE_PREFIX);
    smBlack->Transparent = true;
    smBlack->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smBlue, "smBlue", RESOURCE_PREFIX);
    smBlue->Transparent = true;
    smBlue->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smBrightGreen, "smBrightGreen", RESOURCE_PREFIX);
    smBrightGreen->Transparent = true;
    smBrightGreen->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smCaramel, "smCaramel", RESOURCE_PREFIX);
    smCaramel->Transparent = true;
    smCaramel->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smCyan, "smCyan", RESOURCE_PREFIX);
    smCyan->Transparent = true;
    smCyan->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smLC, "smLC", RESOURCE_PREFIX);
    smLC->Transparent = true;
    smLC->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smLightBlue, "smLightBlue", RESOURCE_PREFIX);
    smLightBlue->Transparent = true;
    smLightBlue->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smMagenta, "smMagenta", RESOURCE_PREFIX);
    smMagenta->Transparent = true;
    smMagenta->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smName, "smName", RESOURCE_PREFIX);
    smName->Transparent = true;
    smName->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smOrange, "smOrange", RESOURCE_PREFIX);
    smOrange->Transparent = true;
    smOrange->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smPaleGreen, "smPaleGreen", RESOURCE_PREFIX);
    smPaleGreen->Transparent = true;
    smPaleGreen->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smRed, "smRed", RESOURCE_PREFIX);
    smRed->Transparent = true;
    smRed->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smYellow, "smYellow", RESOURCE_PREFIX);
    smYellow->Transparent = true;
    smYellow->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(smTransparent, "smTransparent", RESOURCE_PREFIX);
    smTransparent->Transparent = true;
    smTransparent->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(TempBackground, "TempBackground", RESOURCE_PREFIX);
    TempBackground->Transparent = true;
    TempBackground->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(TempHeadCode, "TempHeadCode", RESOURCE_PREFIX);
    TempHeadCode->Transparent = true;
    TempHeadCode->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(UnderHFootbridge, "UnderHFootbridge", RESOURCE_PREFIX);
    UnderHFootbridge->Transparent = true;
    UnderHFootbridge->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(UnderVFootbridge, "UnderVFootbridge", RESOURCE_PREFIX);
    UnderVFootbridge->Transparent = true;
    UnderVFootbridge->TransparentColor = clB5G5R5;

    //failed signal graphics at v2.13.0
    loadResourceFromPrefix(FSig68, "FSig68", RESOURCE_PREFIX);
    FSig68->Transparent = true;
    FSig68->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FSig69, "FSig69", RESOURCE_PREFIX);
    FSig69->Transparent = true;
    FSig69->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FSig70, "FSig70", RESOURCE_PREFIX);
    FSig70->Transparent = true;
    FSig70->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FSig71, "FSig71", RESOURCE_PREFIX);
    FSig71->Transparent = true;
    FSig71->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FSig72, "FSig72", RESOURCE_PREFIX);
    FSig72->Transparent = true;
    FSig72->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FSig73, "FSig73", RESOURCE_PREFIX);
    FSig73->Transparent = true;
    FSig73->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FSig74, "FSig74", RESOURCE_PREFIX);
    FSig74->Transparent = true;
    FSig74->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FSig75, "FSig75", RESOURCE_PREFIX);
    FSig75->Transparent = true;
    FSig75->TransparentColor = clB5G5R5;

    loadResourceFromPrefix(FGSig68, "FGSig68", RESOURCE_PREFIX);
    FGSig68->Transparent = true;
    FGSig68->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FGSig69, "FGSig69", RESOURCE_PREFIX);
    FGSig69->Transparent = true;
    FGSig69->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FGSig70, "FGSig70", RESOURCE_PREFIX);
    FGSig70->Transparent = true;
    FGSig70->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FGSig71, "FGSig71", RESOURCE_PREFIX);
    FGSig71->Transparent = true;
    FGSig71->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FGSig72, "FGSig72", RESOURCE_PREFIX);
    FGSig72->Transparent = true;
    FGSig72->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FGSig73, "FGSig73", RESOURCE_PREFIX);
    FGSig73->Transparent = true;
    FGSig73->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FGSig74, "FGSig74", RESOURCE_PREFIX);
    FGSig74->Transparent = true;
    FGSig74->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(FGSig75, "FGSig75", RESOURCE_PREFIX);
    FGSig75->Transparent = true;
    FGSig75->TransparentColor = clB5G5R5;

    // extra from bmSolidBgnd bitmap file but transparent
    loadResourceFromPrefix(bmTransparentBgnd, "bmSolidBgnd", RESOURCE_PREFIX);
    bmTransparentBgnd->Transparent = true;
    bmTransparentBgnd->TransparentColor = clB5G5R5;

    // level crossing graphics
    loadResourceFromPrefix(LCBothHor, "LCBothHor", RESOURCE_PREFIX);
    LCBothHor->Transparent = true;
    LCBothHor->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCBothHorMan, "LCBothHorMan", RESOURCE_PREFIX);
    LCBothHorMan->Transparent = true;
    LCBothHorMan->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCBotHor, "LCBotHor", RESOURCE_PREFIX);
    LCBotHor->Transparent = true;
    LCBotHor->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCBotHorMan, "LCBotHorMan", RESOURCE_PREFIX);
    LCBotHorMan->Transparent = true;
    LCBotHorMan->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCBothVer, "LCBothVer", RESOURCE_PREFIX);
    LCBothVer->Transparent = true;
    LCBothVer->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCBothVerMan, "LCBothVerMan", RESOURCE_PREFIX);
    LCBothVerMan->Transparent = true;
    LCBothVerMan->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCLHSVer, "LCLHSVer", RESOURCE_PREFIX);
    LCLHSVer->Transparent = true;
    LCLHSVer->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCLHSVerMan, "LCLHSVerMan", RESOURCE_PREFIX);
    LCLHSVerMan->Transparent = true;
    LCLHSVerMan->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCPlain, "LCPlain", RESOURCE_PREFIX);
    LCPlain->Transparent = true;
    LCPlain->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCPlainMan, "LCPlainMan", RESOURCE_PREFIX);
    LCPlainMan->Transparent = true;
    LCPlainMan->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCRHSVer, "LCRHSVer", RESOURCE_PREFIX);
    LCRHSVer->Transparent = true;
    LCRHSVer->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCRHSVerMan, "LCRHSVerMan", RESOURCE_PREFIX);
    LCRHSVerMan->Transparent = true;
    LCRHSVerMan->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCTopHor, "LCTopHor", RESOURCE_PREFIX);
    LCTopHor->Transparent = true;
    LCTopHor->TransparentColor = clB5G5R5;
    loadResourceFromPrefix(LCTopHorMan, "LCTopHorMan", RESOURCE_PREFIX);
    LCTopHorMan->Transparent = true;
    LCTopHorMan->TransparentColor = clB5G5R5;

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
    loadResourceFromPrefix(Code_a, "Code_a", RESOURCE_PREFIX);
    Code_a->Transparent = false;
    loadResourceFromPrefix(Code_b, "Code_b", RESOURCE_PREFIX);
    Code_b->Transparent = false;
    loadResourceFromPrefix(Code_c, "Code_c", RESOURCE_PREFIX);
    Code_c->Transparent = false;
    loadResourceFromPrefix(Code_d, "Code_d", RESOURCE_PREFIX);
    Code_d->Transparent = false;
    loadResourceFromPrefix(Code_e, "Code_e", RESOURCE_PREFIX);
    Code_e->Transparent = false;
    loadResourceFromPrefix(Code_f, "Code_f", RESOURCE_PREFIX);
    Code_f->Transparent = false;
    loadResourceFromPrefix(Code_g, "Code_g", RESOURCE_PREFIX);
    Code_g->Transparent = false;
    loadResourceFromPrefix(Code_h, "Code_h", RESOURCE_PREFIX);
    Code_h->Transparent = false;
    loadResourceFromPrefix(Code_i, "Code_i", RESOURCE_PREFIX);
    Code_i->Transparent = false;
    loadResourceFromPrefix(Code_j, "Code_j", RESOURCE_PREFIX);
    Code_j->Transparent = false;
    loadResourceFromPrefix(Code_k, "Code_k", RESOURCE_PREFIX);
    Code_k->Transparent = false;
    loadResourceFromPrefix(Code_l, "Code_l", RESOURCE_PREFIX);
    Code_l->Transparent = false;
    loadResourceFromPrefix(Code_m, "Code_m", RESOURCE_PREFIX);
    Code_m->Transparent = false;
    loadResourceFromPrefix(Code_n, "Code_n", RESOURCE_PREFIX);
    Code_n->Transparent = false;
    loadResourceFromPrefix(Code_o, "Code_o", RESOURCE_PREFIX);
    Code_o->Transparent = false;
    loadResourceFromPrefix(Code_p, "Code_p", RESOURCE_PREFIX);
    Code_p->Transparent = false;
    loadResourceFromPrefix(Code_q, "Code_q", RESOURCE_PREFIX);
    Code_q->Transparent = false;
    loadResourceFromPrefix(Code_r, "Code_r", RESOURCE_PREFIX);
    Code_r->Transparent = false;
    loadResourceFromPrefix(Code_s, "Code_s", RESOURCE_PREFIX);
    Code_s->Transparent = false;
    loadResourceFromPrefix(Code_t, "Code_t", RESOURCE_PREFIX);
    Code_t->Transparent = false;
    loadResourceFromPrefix(Code_u, "Code_u", RESOURCE_PREFIX);
    Code_u->Transparent = false;
    loadResourceFromPrefix(Code_v, "Code_v", RESOURCE_PREFIX);
    Code_v->Transparent = false;
    loadResourceFromPrefix(Code_w, "Code_w", RESOURCE_PREFIX);
    Code_w->Transparent = false;
    loadResourceFromPrefix(Code_x, "Code_x", RESOURCE_PREFIX);
    Code_x->Transparent = false;
    loadResourceFromPrefix(Code_y, "Code_y", RESOURCE_PREFIX);
    Code_y->Transparent = false;
    loadResourceFromPrefix(Code_z, "Code_z", RESOURCE_PREFIX);
    Code_z->Transparent = false;
    loadResourceFromPrefix(Code0, "Code0", RESOURCE_PREFIX);
    Code0->Transparent = false;
    loadResourceFromPrefix(Code1, "Code1", RESOURCE_PREFIX);
    Code1->Transparent = false;
    loadResourceFromPrefix(Code2, "Code2", RESOURCE_PREFIX);
    Code2->Transparent = false;
    loadResourceFromPrefix(Code3, "Code3", RESOURCE_PREFIX);
    Code3->Transparent = false;
    loadResourceFromPrefix(Code4, "Code4", RESOURCE_PREFIX);
    Code4->Transparent = false;
    loadResourceFromPrefix(Code5, "Code5", RESOURCE_PREFIX);
    Code5->Transparent = false;
    loadResourceFromPrefix(Code6, "Code6", RESOURCE_PREFIX);
    Code6->Transparent = false;
    loadResourceFromPrefix(Code7, "Code7", RESOURCE_PREFIX);
    Code7->Transparent = false;
    loadResourceFromPrefix(Code8, "Code8", RESOURCE_PREFIX);
    Code8->Transparent = false;
    loadResourceFromPrefix(Code9, "Code9", RESOURCE_PREFIX);
    Code9->Transparent = false;
    loadResourceFromPrefix(CodeA, "CodeA", RESOURCE_PREFIX);
    CodeA->Transparent = false;
    loadResourceFromPrefix(CodeB, "CodeB", RESOURCE_PREFIX);
    CodeB->Transparent = false;
    loadResourceFromPrefix(CodeC, "CodeC", RESOURCE_PREFIX);
    CodeC->Transparent = false;
    loadResourceFromPrefix(CodeD, "CodeD", RESOURCE_PREFIX);
    CodeD->Transparent = false;
    loadResourceFromPrefix(CodeE, "CodeE", RESOURCE_PREFIX);
    CodeE->Transparent = false;
    loadResourceFromPrefix(CodeF, "CodeF", RESOURCE_PREFIX);
    CodeF->Transparent = false;
    loadResourceFromPrefix(CodeG, "CodeG", RESOURCE_PREFIX);
    CodeG->Transparent = false;
    loadResourceFromPrefix(CodeH, "CodeH", RESOURCE_PREFIX);
    CodeH->Transparent = false;
    loadResourceFromPrefix(CodeI, "CodeI", RESOURCE_PREFIX);
    CodeI->Transparent = false;
    loadResourceFromPrefix(CodeJ, "CodeJ", RESOURCE_PREFIX);
    CodeJ->Transparent = false;
    loadResourceFromPrefix(CodeK, "CodeK", RESOURCE_PREFIX);
    CodeK->Transparent = false;
    loadResourceFromPrefix(CodeL, "CodeL", RESOURCE_PREFIX);
    CodeL->Transparent = false;
    loadResourceFromPrefix(CodeM, "CodeM", RESOURCE_PREFIX);
    CodeM->Transparent = false;
    loadResourceFromPrefix(CodeN, "CodeN", RESOURCE_PREFIX);
    CodeN->Transparent = false;
    loadResourceFromPrefix(CodeO, "CodeO", RESOURCE_PREFIX);
    CodeO->Transparent = false;
    loadResourceFromPrefix(CodeP, "CodeP", RESOURCE_PREFIX);
    CodeP->Transparent = false;
    loadResourceFromPrefix(CodeQ, "CodeQ", RESOURCE_PREFIX);
    CodeQ->Transparent = false;
    loadResourceFromPrefix(CodeR, "CodeR", RESOURCE_PREFIX);
    CodeR->Transparent = false;
    loadResourceFromPrefix(CodeS, "CodeS", RESOURCE_PREFIX);
    CodeS->Transparent = false;
    loadResourceFromPrefix(CodeT, "CodeT", RESOURCE_PREFIX);
    CodeT->Transparent = false;
    loadResourceFromPrefix(CodeU, "CodeU", RESOURCE_PREFIX);
    CodeU->Transparent = false;
    loadResourceFromPrefix(CodeV, "CodeV", RESOURCE_PREFIX);
    CodeV->Transparent = false;
    loadResourceFromPrefix(CodeW, "CodeW", RESOURCE_PREFIX);
    CodeW->Transparent = false;
    loadResourceFromPrefix(CodeX, "CodeX", RESOURCE_PREFIX);
    CodeX->Transparent = false;
    loadResourceFromPrefix(CodeY, "CodeY", RESOURCE_PREFIX);
    CodeY->Transparent = false;
    loadResourceFromPrefix(CodeZ, "CodeZ", RESOURCE_PREFIX);
    CodeZ->Transparent = false;
    loadResourceFromPrefix(bmSolidBgnd, "bmSolidBgnd", RESOURCE_PREFIX);
    bmSolidBgnd->Transparent = false;
    loadResourceFromPrefix(smSolidBgnd, "smSolidBgnd", RESOURCE_PREFIX);
    smSolidBgnd->Transparent = false;
    loadResourceFromPrefix(bmDiagonalSignalBlank, "bmDiagonalSignalBlank", RESOURCE_PREFIX);
    bmDiagonalSignalBlank->Transparent = false;
    loadResourceFromPrefix(bmPointBlank, "bmPointBlank", RESOURCE_PREFIX);
    bmPointBlank->Transparent = false;
    loadResourceFromPrefix(bmStraightEWSignalBlank, "bmStraightEWSignalBlank", RESOURCE_PREFIX);
    bmStraightEWSignalBlank->Transparent = false;
    loadResourceFromPrefix(bmStraightNSSignalBlank, "bmStraightNSSignalBlank", RESOURCE_PREFIX);
    bmStraightNSSignalBlank->Transparent = false;
}

// ---------------------------------------------------------------------------


void TRailGraphics::ChangeTransparentColour(Graphics::TBitmap *BitmapIn, Graphics::TBitmap *BitmapOut, TColor NewTransparentColour, TColor OldTransparentColour)
{
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

void TRailGraphics::ChangeAllTransparentColours(TColor NewTransparentColour, TColor OldTransparentColour)
{
    if(NewTransparentColour == OldTransparentColour)
    {
        return; // already stored

    }
    ChangeTransparentColour(BlackCircle, BlackCircle, NewTransparentColour, OldTransparentColour); //added at v2.13.0
    ChangeTransparentColour(bm10, bm10, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm100, bm100, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm101, bm101, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm106, bm106, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm10Diverging, bm10Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm10Straight, bm10Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm11, bm11, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm11Diverging, bm11Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm11Straight, bm11Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm12, bm12, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm12Diverging, bm12Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm12Straight, bm12Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm13, bm13, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm132, bm132, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm132LeftFork, bm132LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm132RightFork, bm132RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm133, bm133, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm133LeftFork, bm133LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm133RightFork, bm133RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm134, bm134, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm134LeftFork, bm134LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm134RightFork, bm134RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm135, bm135, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm135LeftFork, bm135LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm135RightFork, bm135RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm136, bm136, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm136LeftFork, bm136LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm136RightFork, bm136RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm137, bm137, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm137LeftFork, bm137LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm137RightFork, bm137RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm138, bm138, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm138LeftFork, bm138LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm138RightFork, bm138RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm139, bm139, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm139LeftFork, bm139LeftFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm139RightFork, bm139RightFork, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm13Diverging, bm13Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm13Straight, bm13Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm14, bm14, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm140, bm140, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm141, bm141, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm14Diverging, bm14Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm14Straight, bm14Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm16, bm16, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm18, bm18, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm20, bm20, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm27, bm27, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm28, bm28, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm28Diverging, bm28Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm28Straight, bm28Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm29, bm29, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm29Diverging, bm29Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm29Straight, bm29Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm30, bm30, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm30Diverging, bm30Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm30Straight, bm30Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm31, bm31, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm31Diverging, bm31Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm31Straight, bm31Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm32, bm32, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm32Diverging, bm32Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm32Straight, bm32Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm33, bm33, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm33Diverging, bm33Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm33Straight, bm33Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm34, bm34, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm34Diverging, bm34Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm34Straight, bm34Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm35, bm35, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm35Diverging, bm35Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm35Straight, bm35Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm36, bm36, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm36Diverging, bm36Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm36Straight, bm36Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm37, bm37, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm37Diverging, bm37Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm37Straight, bm37Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm38, bm38, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm38Diverging, bm38Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm38Straight, bm38Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm39, bm39, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm39Diverging, bm39Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm39Straight, bm39Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm40, bm40, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm40Diverging, bm40Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm40Straight, bm40Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm41, bm41, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm41Diverging, bm41Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm41Straight, bm41Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm42, bm42, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm42Diverging, bm42Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm42Straight, bm42Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm43, bm43, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm43Diverging, bm43Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm43Straight, bm43Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm45, bm45, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm46, bm46, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm50, bm50, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm51, bm51, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm53, bm53, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm54, bm54, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm56, bm56, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm59, bm59, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm65, bm65, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm68CallingOn, bm68CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm68dblyellow, bm68dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm68grounddblred, bm68grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm68grounddblwhite, bm68grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm68green, bm68green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm68yellow, bm68yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm69CallingOn, bm69CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm69dblyellow, bm69dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm69grounddblred, bm69grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm69grounddblwhite, bm69grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm69green, bm69green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm69yellow, bm69yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm7, bm7, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm70CallingOn, bm70CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm70dblyellow, bm70dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm70grounddblred, bm70grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm70grounddblwhite, bm70grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm70green, bm70green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm70yellow, bm70yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm71CallingOn, bm71CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm71dblyellow, bm71dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm71grounddblred, bm71grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm71grounddblwhite, bm71grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm71green, bm71green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm71yellow, bm71yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm72CallingOn, bm72CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm72dblyellow, bm72dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm72grounddblred, bm72grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm72grounddblwhite, bm72grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm72green, bm72green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm72yellow, bm72yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm73, bm73, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm73CallingOn, bm73CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm73dblyellow, bm73dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm73grounddblred, bm73grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm73grounddblwhite, bm73grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm73green, bm73green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm73yellow, bm73yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm74, bm74, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm74CallingOn, bm74CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm74dblyellow, bm74dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm74grounddblred, bm74grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm74grounddblwhite, bm74grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm74green, bm74green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm74yellow, bm74yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm75CallingOn, bm75CallingOn, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm75dblyellow, bm75dblyellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm75grounddblred, bm75grounddblred, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm75grounddblwhite, bm75grounddblwhite, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm75green, bm75green, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm75yellow, bm75yellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm77, bm77, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm77Striped, bm77Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm78, bm78, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm78Striped, bm78Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm7Diverging, bm7Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm7Straight, bm7Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm8, bm8, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm85, bm85, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm8Diverging, bm8Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm8Straight, bm8Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm9, bm9, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm93set, bm93set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm93unset, bm93unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm94set, bm94set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm94unset, bm94unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm9Diverging, bm9Diverging, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bm9Straight, bm9Straight, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmGreenEllipse, bmGreenEllipse, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmGreenRect, bmGreenRect, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmGrid, bmGrid, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmLightBlueRect, bmLightBlueRect, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmName, bmName, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmNameStriped, bmNameStriped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRedEllipse, bmRedEllipse, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRedRect, bmRedRect, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink1, bmRtCancELink1, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink2, bmRtCancELink2, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink3, bmRtCancELink3, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink4, bmRtCancELink4, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink6, bmRtCancELink6, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink7, bmRtCancELink7, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink8, bmRtCancELink8, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmRtCancELink9, bmRtCancELink9, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br1, br1, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br10, br10, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br11, br11, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br12, br12, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br2, br2, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br3, br3, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br4, br4, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br5, br5, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br6, br6, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br7, br7, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br8, br8, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(br9, br9, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Concourse, Concourse, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ConcourseGlyph, ConcourseGlyph, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ConcourseStriped, ConcourseStriped, NewTransparentColour, OldTransparentColour);

    ChangeTransparentColour(CouplingExit1, CouplingExit1, NewTransparentColour, OldTransparentColour); //Multiplayer coupled exit graphics
    ChangeTransparentColour(CouplingExit2, CouplingExit2, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(CouplingExit3, CouplingExit3, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(CouplingExit4, CouplingExit4, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(CouplingExit6, CouplingExit6, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(CouplingExit7, CouplingExit7, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(CouplingExit8, CouplingExit8, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(CouplingExit9, CouplingExit9, NewTransparentColour, OldTransparentColour);

    ChangeTransparentColour(ELk1, ELk1, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ELk2, ELk2, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ELk3, ELk3, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ELk4, ELk4, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ELk6, ELk6, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ELk7, ELk7, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ELk8, ELk8, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(ELk9, ELk9, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(BlackOctagon, BlackOctagon, NewTransparentColour, OldTransparentColour); //added at v2.13.0
    ChangeTransparentColour(gl1, gl1, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl10, gl10, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl100, gl100, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl101, gl101, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl102, gl102, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl103, gl103, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl104, gl104, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl105, gl105, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl106, gl106, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl107, gl107, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl108, gl108, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl109, gl109, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl11, gl11, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl110, gl110, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl111, gl111, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl112, gl112, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl113, gl113, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl114, gl114, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl115, gl115, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl116, gl116, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl117, gl117, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl118, gl118, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl119, gl119, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl12, gl12, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl120, gl120, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl121, gl121, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl122, gl122, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl123, gl123, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl124, gl124, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl125, gl125, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl126, gl126, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl127, gl127, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl128, gl128, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl129, gl129, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl129Striped, gl129Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl13, gl13, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl130, gl130, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl130Striped, gl130Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl131, gl131, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl132, gl132, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl133, gl133, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl134, gl134, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl135, gl135, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl136, gl136, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl137, gl137, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl138, gl138, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl139, gl139, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl14, gl14, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl140, gl140, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl141, gl141, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl142, gl142, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl143, gl143, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl145, gl145, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl145Striped, gl145Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl146, gl146, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl146Striped, gl146Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl15, gl15, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl16, gl16, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl18, gl18, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl19, gl19, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl2, gl2, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl20, gl20, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl21, gl21, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl22, gl22, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl23, gl23, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl24, gl24, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl25, gl25, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl26, gl26, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl27, gl27, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl28, gl28, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl29, gl29, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl3, gl3, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl30, gl30, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl31, gl31, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl32, gl32, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl33, gl33, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl34, gl34, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl35, gl35, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl36, gl36, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl37, gl37, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl38, gl38, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl39, gl39, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl4, gl4, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl40, gl40, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl41, gl41, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl42, gl42, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl43, gl43, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl44, gl44, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl45, gl45, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl46, gl46, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl47, gl47, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl48, gl48, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl49, gl49, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl5, gl5, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl50, gl50, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl51, gl51, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl52, gl52, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl53, gl53, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl54, gl54, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl55, gl55, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl56, gl56, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl57, gl57, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl58, gl58, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl59, gl59, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl6, gl6, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl60, gl60, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl61, gl61, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl62, gl62, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl63, gl63, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl64, gl64, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl65, gl65, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl66, gl66, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl67, gl67, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl68, gl68, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl69, gl69, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl7, gl7, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl70, gl70, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl71, gl71, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl72, gl72, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl73, gl73, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl74, gl74, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl75, gl75, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl76, gl76, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl76Striped, gl76Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl77, gl77, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl78, gl78, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl79, gl79, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl79Striped, gl79Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl8, gl8, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl80, gl80, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl81, gl81, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl82, gl82, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl83, gl83, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl84, gl84, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl85, gl85, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl86, gl86, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl87, gl87, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl88set, gl88set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl88unset, gl88unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl89set, gl89set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl89unset, gl89unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl9, gl9, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl90set, gl90set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl90unset, gl90unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl91set, gl91set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl91unset, gl91unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl92set, gl92set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl92unset, gl92unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl93set, gl93set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl94set, gl94set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl95set, gl95set, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl95unset, gl95unset, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl97, gl97, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl98, gl98, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(gl99, gl99, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat68, Plat68, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat68Striped, Plat68Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat69, Plat69, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat69Striped, Plat69Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat70, Plat70, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat70Striped, Plat70Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat71, Plat71, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(Plat71Striped, Plat71Striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm1, sm1, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm10, sm10, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm100, sm100, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm101, sm101, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm102, sm102, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm103, sm103, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm104, sm104, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm105, sm105, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm106, sm106, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm107, sm107, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm108, sm108, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm109, sm109, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm11, sm11, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm110, sm110, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm111, sm111, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm112, sm112, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm115, sm115, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm117, sm117, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm12, sm12, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm129, sm129, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm129striped, sm129striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm13, sm13, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm130, sm130, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm130striped, sm130striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm131striped, sm131striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm132, sm132, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm133, sm133, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm134, sm134, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm135, sm135, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm136, sm136, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm137, sm137, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm138, sm138, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm139, sm139, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm14, sm14, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm15, sm15, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm16, sm16, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm18, sm18, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm19, sm19, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm2, sm2, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm20, sm20, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm21, sm21, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm22, sm22, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm23, sm23, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm24, sm24, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm25, sm25, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm26, sm26, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm27, sm27, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm28, sm28, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm29, sm29, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm3, sm3, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm30, sm30, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm31, sm31, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm32, sm32, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm33, sm33, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm34, sm34, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm35, sm35, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm36, sm36, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm37, sm37, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm38, sm38, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm39, sm39, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm4, sm4, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm40, sm40, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm41, sm41, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm42, sm42, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm43, sm43, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm44, sm44, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm45, sm45, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm46, sm46, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm47, sm47, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm48, sm48, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm49, sm49, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm5, sm5, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm50, sm50, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm51, sm51, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm52, sm52, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm53, sm53, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm54, sm54, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm55, sm55, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm56, sm56, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm57, sm57, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm58, sm58, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm59, sm59, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm6, sm6, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm60, sm60, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm61, sm61, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm62, sm62, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm63, sm63, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm64, sm64, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm65, sm65, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm66, sm66, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm67, sm67, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm7, sm7, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm76, sm76, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm76striped, sm76striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm77, sm77, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm77striped, sm77striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm78, sm78, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm78striped, sm78striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm79, sm79, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm79striped, sm79striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm8, sm8, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm80, sm80, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm81, sm81, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm82, sm82, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm83, sm83, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm84, sm84, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm85, sm85, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm86, sm86, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm87, sm87, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm88, sm88, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm89, sm89, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm9, sm9, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm90, sm90, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm91, sm91, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm92, sm92, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm93, sm93, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm94, sm94, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm95, sm95, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm96, sm96, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm96striped, sm96striped, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm97, sm97, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm98, sm98, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(sm99, sm99, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smBlack, smBlack, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smBlue, smBlue, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smBrightGreen, smBrightGreen, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smCaramel, smCaramel, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smCyan, smCyan, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smLightBlue, smLightBlue, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smMagenta, smMagenta, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smName, smName, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smPaleGreen, smPaleGreen, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smRed, smRed, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smYellow, smYellow, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smTransparent, smTransparent, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(TempBackground, TempBackground, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(TempHeadCode, TempHeadCode, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(UnderHFootbridge, UnderHFootbridge, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(UnderVFootbridge, UnderVFootbridge, NewTransparentColour, OldTransparentColour);

    ChangeTransparentColour(FSig68, FSig68, NewTransparentColour, OldTransparentColour); //failed signals, added at v2.13.0
    ChangeTransparentColour(FSig69, FSig69, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FSig70, FSig70, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FSig71, FSig71, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FSig72, FSig72, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FSig73, FSig73, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FSig74, FSig74, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FSig75, FSig75, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig68, FGSig68, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig69, FGSig69, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig70, FGSig70, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig71, FGSig71, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig72, FGSig72, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig73, FGSig73, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig74, FGSig74, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(FGSig75, FGSig75, NewTransparentColour, OldTransparentColour);

    // The following are created as new bitmaps from existing  files
    ChangeTransparentColour(bmTransparentBgnd, bmTransparentBgnd, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(GridBitmap, GridBitmap, NewTransparentColour, OldTransparentColour);

    // non-transparent graphics - don't change headcodes
    ChangeTransparentColour(bmSolidBgnd, bmSolidBgnd, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(smSolidBgnd, smSolidBgnd, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmDiagonalSignalBlank, bmDiagonalSignalBlank, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmPointBlank, bmPointBlank, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmStraightEWSignalBlank, bmStraightEWSignalBlank, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(bmStraightNSSignalBlank, bmStraightNSSignalBlank, NewTransparentColour, OldTransparentColour);

    // level crossing graphics
    ChangeTransparentColour(LCBothHor, LCBothHor, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCBotHor, LCBotHor, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCBothVer, LCBothVer, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCLHSVer, LCLHSVer, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCPlain, LCPlain, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCRHSVer, LCRHSVer, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCTopHor, LCTopHor, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCBothHorMan, LCBothHorMan, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCBotHorMan, LCBotHorMan, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCBothVerMan, LCBothVerMan, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCLHSVerMan, LCLHSVerMan, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCPlainMan, LCPlainMan, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCRHSVerMan, LCRHSVerMan, NewTransparentColour, OldTransparentColour);
    ChangeTransparentColour(LCTopHorMan, LCTopHorMan, NewTransparentColour, OldTransparentColour);

// change the grid to the nearest grey colour to the background
    if(NewTransparentColour != clB5G5R5)
    {
        ChangeSpecificColour(1, GridBitmap, GridBitmap, clB4G4R4, clB1G1R1); // if already dark will ignore
    }
    else
    {
        ChangeSpecificColour(3, GridBitmap, GridBitmap, clB1G1R1, clB4G4R4); // if already light will ignore
    }
}

// ---------------------------------------------------------------------------

void TRailGraphics::SetUpAllDerivitiveGraphics(TColor TransparentColour)
{
    for(int x = 0; x < 30; x++)
    {
        LinkPrefDirGraphicsPtr[x]->TransparentColor = TransparentColour;
        LinkNonSigRouteGraphicsPtr[x]->TransparentColor = TransparentColour;
        LinkSigRouteGraphicsPtr[x]->TransparentColor = TransparentColour;
        LinkRouteAutoSigsGraphicsPtr[x]->TransparentColor = TransparentColour;
    }
    for(int x = 0; x < 12; x++)
    {
        BridgePrefDirGraphicsPtr[x]->TransparentColor = TransparentColour;
        BridgeNonSigRouteGraphicsPtr[x]->TransparentColor = TransparentColour;
        BridgeSigRouteGraphicsPtr[x]->TransparentColor = TransparentColour;
        BridgeRouteAutoSigsGraphicsPtr[x]->TransparentColor = TransparentColour;
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
        ChangeForegroundColour(5, LinkGraphicsPtr[x], LinkPrefDirGraphicsPtr[x], clB2G0R4, TransparentColour); // magenta
        ChangeForegroundColour(6, LinkGraphicsPtr[x], LinkNonSigRouteGraphicsPtr[x], clB0G0R5, TransparentColour); // red
        ChangeForegroundColour(7, LinkGraphicsPtr[x], LinkSigRouteGraphicsPtr[x], clB0G4R0, TransparentColour); // green
        ChangeForegroundColour(8, LinkGraphicsPtr[x], LinkRouteAutoSigsGraphicsPtr[x], clB5G3R0, TransparentColour); // blue
    }
    for(int x = 0; x < 12; x++)
    {
        ChangeForegroundColour(9, BridgeGraphicsPtr[x], BridgePrefDirGraphicsPtr[x], clB2G0R4, TransparentColour);
        ChangeForegroundColour(10, BridgeGraphicsPtr[x], BridgeNonSigRouteGraphicsPtr[x], clB0G0R5, TransparentColour);
        ChangeForegroundColour(11, BridgeGraphicsPtr[x], BridgeSigRouteGraphicsPtr[x], clB0G4R0, TransparentColour);
        ChangeForegroundColour(12, BridgeGraphicsPtr[x], BridgeRouteAutoSigsGraphicsPtr[x], clB5G3R0, TransparentColour);
    }
    for(int x = 0; x < 10; x++)
    {
        ChangeForegroundColour(13, DirectionPrefDirGraphicsPtr[x], DirectionPrefDirGraphicsPtr[x], clB2G0R4, TransparentColour);
        ChangeForegroundColour(14, DirectionNonSigRouteGraphicsPtr[x], DirectionNonSigRouteGraphicsPtr[x], clB0G0R5, TransparentColour);
        ChangeForegroundColour(15, DirectionSigRouteGraphicsPtr[x], DirectionSigRouteGraphicsPtr[x], clB0G4R0, TransparentColour);
        ChangeForegroundColour(16, DirectionRouteAutoSigsGraphicsPtr[x], DirectionRouteAutoSigsGraphicsPtr[x], clB5G3R0, TransparentColour);
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

    loadResourceFromPrefix(TmpBM, "bmSolidBgnd", RESOURCE_PREFIX);
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

    loadResourceFromPrefix(TmpBM, "bmSolidBgnd", RESOURCE_PREFIX);
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

    loadResourceFromPrefix(TmpBM, "bmSolidBgnd", RESOURCE_PREFIX);
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

    loadResourceFromPrefix(TmpBM, "bmSolidBgnd", RESOURCE_PREFIX);
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


