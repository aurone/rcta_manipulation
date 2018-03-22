////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2014, Andrew Dornbush All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#ifndef au_ConfigBlock_h
#define au_ConfigBlock_h

#include <memory>
#include <string>
#include <vector>
#include <istream>
#include <lua.hpp>

namespace au
{

class ConfigBlock
{
public:

    ConfigBlock();
    ~ConfigBlock();

    // disallow copy and assign
    ConfigBlock(const ConfigBlock&) = delete;
    ConfigBlock& operator=(const ConfigBlock&) = delete;

    /// @brief Load the ConfigBlock from the global value @confparam in the Lua program represented by @luaconf
    bool load(const std::string& luaconf, const std::string& confparam);

    bool get(const std::string& param, int& val);

    bool get(const std::string& param, double& val);

    bool get(const std::string& param, bool& val);

    bool get(const std::string& param, std::string& val);

    bool get(const std::string& param, ConfigBlock& config);

private:

    typedef std::unique_ptr<lua_State, void (*)(lua_State*)> LuaStatePtr;

    // maintain the source that was used to initialize lua so that we can clone the lua state
    std::string luaconf_;               

    // for some reason i decided to keep the stack of all the table names insteaad of just cloning the lua stack directly which is probably kinda maybe sorta doable
    std::vector<std::string> tables_;   

    // each config block maintains its own lua state
    LuaStatePtr L_;

    // clone a new lua state
    LuaStatePtr clone_state();

    bool trace_tables(lua_State *L, const std::vector<std::string>& tables);
};

} // namespace au

#endif
