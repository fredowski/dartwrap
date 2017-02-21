package = 'dartwrap'
version = '1.0-1'

source = {
   url = ''
}

description = {
  summary = "Dartwrap"
}

dependencies = { 'paths' }
build = {
   type = "command",
   build_command = [[
cmake -E make_directory build;
cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(LUA_BINDIR)/.." -DCMAKE_INSTALL_PREFIX="$(PREFIX)";
$(MAKE)
   ]],
   install_command = "$(MAKE) -C build install"
}
