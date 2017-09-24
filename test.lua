--[[dartwrap - dart simulator interface for torch
   Copyright (c) 2017, Friedrich Beckmann, Hochschule Augsburg

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
]]

-- run with "th -ldartwrap test.lua"

dw = require 'dartwrap'

local mytest = torch.TestSuite()
local tester = torch.Tester()

function mytest.simple()
   p = dw.newDartSim()
   r = dw.play(p, 5)
   tester:asserteq(r,5, "This should be 5")
   r = dw.play(p, 3)
   tester:asserteq(r,8, "This should be 8")
   for i=1,1000000 do
     r = dw.play(p,18)
   end
end

tester:add(mytest)
tester:run()
