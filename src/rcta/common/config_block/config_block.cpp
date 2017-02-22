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

#include <cassert>
#include "config_block.h"

namespace au
{

static void free_lua_state(lua_State *l)
{
    if (l) {
        lua_close(l);
    }
}

static const char *lua_type_to_str(int lua_type)
{
    switch (lua_type) {
    case LUA_TNUMBER:
        return "Number";
    case LUA_TBOOLEAN:
        return "Boolean";
    case LUA_TSTRING:
        return "String";
    case LUA_TTABLE:
        return "Table";
    case LUA_TNIL:
        return "Nil";
    case LUA_TFUNCTION:
        return "Function";
    case LUA_TUSERDATA:
        return "UserData";
    case LUA_TTHREAD:
        return "Thread";
    default:
        return "Unknown";
    }
}

ConfigBlock::ConfigBlock() :
    luaconf_(),
    L_(nullptr, free_lua_state)
{
}

ConfigBlock::~ConfigBlock()
{
}

bool ConfigBlock::load(const std::string& luaconf, const std::string& confparam)
{
    LuaStatePtr L(luaL_newstate(), free_lua_state);

    if (!L) {
        fprintf(stderr, "Failed to create new Lua state\n");
        return false;
    }

    // load string and compile to chunk
    if (luaL_loadstring(L.get(), luaconf.c_str())) {
        fprintf(stderr, "%s\n", lua_tostring(L.get(), -1));
        return false;
    }

    // run the chunk
    if (lua_pcall(L.get(), 0, 0, 0)) {
        fprintf(stderr, "Failed to parse '%s'\n", luaconf.c_str());
        return false;
    }

    lua_getglobal(L.get(), confparam.c_str());
    if (!lua_istable(L.get(), -1)) {
        fprintf(stderr, "Param '%s' does not meet ConfigBlock requirements (Type = %s)\n",
                confparam.c_str(), lua_type_to_str(lua_type(L.get(), -1)));
        fprintf(stderr, "'%s'\n", lua_tostring(L.get(), -1));
        return false;
    }

    tables_.push_back(confparam);
    L_ = std::move(L);
    luaconf_ = luaconf;
    return true;
}

bool ConfigBlock::get(const std::string& param, int& val)
{
    if (!L_) {
        fprintf(stderr, "Lua state is crap\n");
        return false;
    }

    if (!lua_istable(L_.get(), -1)) {
        fprintf(stderr, "hmm\n");
        return false;
    }

    lua_getfield(L_.get(), -1, param.c_str());
    if (!lua_isnumber(L_.get(), -1)) {
        fprintf(stderr, "Param '%s' is of type '%s'\n", param.c_str(), lua_type_to_str(lua_type(L_.get(), -1)));
        lua_pop(L_.get(), 1);
        return false;
    }

    val = lua_tointeger(L_.get(), -1);
    lua_pop(L_.get(), 1);
    return true;
}

bool ConfigBlock::get(const std::string& param, double& val)
{
    if (!L_) {
        return false;
    }

    lua_getfield(L_.get(), -1, param.c_str());
    if (!lua_isnumber(L_.get(), -1)) {
        lua_pop(L_.get(), 1);
        return false;
    }
    val = lua_tonumber(L_.get(), -1);
    lua_pop(L_.get(), 1);
    return true;
}

bool ConfigBlock::get(const std::string& param, bool& val)
{
    if (!L_) {
        return false;
    }

    lua_getfield(L_.get(), -1, param.c_str());
    if (!lua_isboolean(L_.get(), -1)) {
        lua_pop(L_.get(), 1);
        return false;
    }
    val = (bool)lua_toboolean(L_.get(), -1);
    lua_pop(L_.get(), 1);
    return true;
}

bool ConfigBlock::get(const std::string& param, std::string& val)
{
    if (!L_) {
        return false;
    }

    lua_getfield(L_.get(), -1, param.c_str());
    if (!lua_isstring(L_.get(), -1)) {
        lua_pop(L_.get(), 1);
        return false;
    }
    val = std::string(lua_tostring(L_.get(), -1));
    lua_pop(L_.get(), 1);
    return true;
}

bool ConfigBlock::get(const std::string& param, ConfigBlock& config)
{
    if (!L_) {
        return false;
    }

    // check for a table at param
    lua_getfield(L_.get(), -1, param.c_str());
    if (!lua_istable(L_.get(), -1)) {
        lua_pop(L_.get(), 1);
        return false;
    }

    // clone the state with the table on the top of the stack
    tables_.push_back(param);
    LuaStatePtr new_state = clone_state();
    if (!new_state) {
        fprintf(stderr, "Failed to clone Lua State\n");
        tables_.pop_back();
        lua_pop(L_.get(), 1);
        return false;
    }

    // remove the table from this lua state
    tables_.pop_back();
    lua_pop(L_.get(), 1);

    // pass the cloned state, the new table stack, and the original luaconf
    config.L_ = std::move(new_state);
    config.tables_ = tables_;
    config.tables_.push_back(param);
    config.luaconf_ = luaconf_;
    return true;
}

auto ConfigBlock::clone_state() -> LuaStatePtr
{
    if (luaconf_.empty()) { // successfully compiled string if non-empty
        return LuaStatePtr(nullptr, free_lua_state);
    }

    assert(!tables_.empty() && L_); // state consistent with !luaconf_.empty()

    // create a new lua state
    LuaStatePtr L(luaL_newstate(), free_lua_state);
    if (!L) {
        fprintf(stderr, "Failed to create new Lua state\n");
        return LuaStatePtr(nullptr, free_lua_state);
    }

    // load and compile the chunk
    if (luaL_loadstring(L.get(), luaconf_.c_str())) {
        fprintf(stderr, "%s\n", lua_tostring(L.get(), -1));
        return LuaStatePtr(nullptr, free_lua_state);
    }

    // run the chunk
    if (lua_pcall(L.get(), 0, 0, 0)) {
        fprintf(stderr, "Failed to parse '%s'\n", luaconf_.c_str());
        return LuaStatePtr(nullptr, free_lua_state);
    }

    // trace all the tables, leaving the most recent table on the top of the stack
    if (trace_tables(L.get(), tables_)) {
        return L;
    }
    else {
        return LuaStatePtr(nullptr, free_lua_state);
    }
}

bool ConfigBlock::trace_tables(lua_State *L, const std::vector<std::string>& tables)
{
    // the first table is the global parameter
    lua_getglobal(L, tables_.front().c_str());
    if (!lua_istable(L, -1)) {
        fprintf(stderr, "Param '%s' does not meet ConfigBlock requirements (Type = %s)\n",
                tables_.front().c_str(), lua_type_to_str(lua_type(L, -1)));
        fprintf(stderr, "'%s'\n", lua_tostring(L, -1));
        return false;
    }

    // the rest of the tables are the nested parameter tables
    for (size_t i = 1; i < tables.size(); ++i) {
        lua_getfield(L, -1, tables[i].c_str());
        if (!lua_istable(L, -1)) {
            return false;
        }
    }

    return true;
}

} // namespace au
