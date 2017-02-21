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

require 'paths'

-- Reads the whole content of the specified file.
local function readContent(path)
    local file = io.open(path)
    local content = file:read("*a")
    file:close()
    return content
end

-- Appends all srcList values to the destList.
local function appendAll(destList, srcList)
    for _, val in ipairs(srcList) do
        table.insert(destList, val)
    end
end

local ffi = require 'ffi'
ffi.cdef(readContent(paths.thisfile("dartwrap.inl")))
local lib = ffi.load(package.searchpath('libdartwrap',package.cpath))

-- Creates a new DartSim instance.
function dartwrap.newDartSim()
   return lib.dart_new()
end

function dartwrap.play(dartsim, action)
   return lib.dart_act(dartsim, action)
end
