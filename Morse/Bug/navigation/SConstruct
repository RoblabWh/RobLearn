# vim: set filetype=python:
import os

env = Environment()
EDUMORSEPATH = os.environ.get("EDUMORSEPATH")
CBINDINGSPATH = os.path.join(EDUMORSEPATH, "libraries/cBindings")
env.Append(LIBPATH=CBINDINGSPATH)
env.Append(CPPPATH=CBINDINGSPATH)

t = env.Program('main', 'main.c', LIBS=['cBindings', 'm'])

env.NoClean('main')
