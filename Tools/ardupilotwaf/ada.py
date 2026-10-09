# encoding: utf-8

# AP_FLAKE8_CLEAN

"""
WAF Tool to build the Clearwater Ada/SPARK flight software library
(libcw_fcs.a, from the fcs/ project next to this tree) with gprbuild and
link it into every program. Enabled with ./waf configure --enable-ada.

gprbuild runs before any C++ is compiled, on every build; it is incremental
itself. Programs relink when the library's contents change.
"""

import glob
import os

from waflib import Options, Task, Utils
from waflib.Configure import conf
from waflib.TaskGen import after_method, feature

# board class -> CW_TARGET of fcs.gpr
_TARGETS = {
    'SITL': 'native',
    'ChibiOS': 'arm',
}


def options(opt):
    g = opt.ap_groups['configure']
    g.add_option('--enable-ada', action='store_true', default=False,
                 help='Build and link the Ada/SPARK flight software (AP_Ada)')
    g.add_option('--fcs-dir', default=None,
                 help='Path of the fcs Ada project (default: ../fcs)')


def configure(cfg):
    cfg.env.AP_ADA_ENABLED = False
    cfg.start_msg('Ada flight software')
    if not cfg.options.enable_ada:
        cfg.end_msg('disabled', color='YELLOW')
        return

    fcs_dir = cfg.options.fcs_dir or os.path.join(cfg.srcnode.abspath(), '..', 'fcs')
    fcs_dir = os.path.abspath(fcs_dir)
    if not os.path.exists(os.path.join(fcs_dir, 'fcs.gpr')):
        cfg.fatal('fcs.gpr not found in %s, set --fcs-dir' % fcs_dir)
    target = _TARGETS.get(cfg.env.BOARD_CLASS)
    if target is None:
        cfg.fatal('Ada is not supported on board class %s' % cfg.env.BOARD_CLASS)
    cfg.end_msg('%s (%s)' % (fcs_dir, target))

    # Alire's toolchain directories hold their own gcc, g++ and binutils, so
    # they must not go on the PATH of the C++ build: find GNAT there (or on
    # PATH) and give its directory to gprbuild only
    alire = glob.glob(os.path.expanduser('~/.local/share/alire/toolchains/*/bin'))
    alire.append(os.path.expanduser('~/.alire/bin'))
    search = os.environ.get('PATH', '').split(os.pathsep) + alire
    gprbuild = cfg.find_program('gprbuild', var='GPRBUILD', path_list=search)
    gnat = cfg.find_program('gnat' if target == 'native' else 'arm-eabi-gnat',
                            var='FCS_GNAT', path_list=search)
    cfg.env.FCS_PATH = [os.path.dirname(gnat[0]), os.path.dirname(gprbuild[0])]

    cfg.env.AP_ADA_ENABLED = True
    cfg.env.FCS_DIR = fcs_dir
    cfg.env.FCS_TARGET = target
    cfg.env.prepend_value('INCLUDES', [os.path.join(fcs_dir, 'include')])
    cfg.define('AP_ADA_ENABLED', 1)


class gprbuild_fcs(Task.Task):
    '''build libcw_fcs.a; gprbuild decides what is out of date'''
    always_run = True
    color = 'BLUE'

    def keyword(self):
        return 'Building Ada'

    def __str__(self):
        return 'fcs.gpr (%s)' % self.env.FCS_TARGET

    def run(self):
        out_dir = self.outputs[0].parent.parent.parent.abspath()
        # as many processes as waf's own -j: the laptop has a load cap
        cmd = self.env.GPRBUILD + [
            '-p', '-q', '-j%d' % Options.options.jobs,
            '-P', os.path.join(self.env.FCS_DIR, 'fcs.gpr'),
            '-XCW_TARGET=%s' % self.env.FCS_TARGET,
            '-XCW_BUILD=release',
            '-XCW_OUT_DIR=%s' % out_dir,
        ]
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join(self.env.FCS_PATH + [env.get('PATH', '')])
        return self.exec_command(cmd, env=env)

    def post_run(self):
        super(gprbuild_fcs, self).post_run()
        # sign the library by its contents, so that programs relink only
        # when the Ada code changed (the task itself always runs)
        for node in self.outputs:
            node.sig = Utils.h_file(node.abspath())


@conf
def ada_fcs(bld):
    lib = bld.bldnode.find_or_declare(
        'fcs/lib/%s-release/libcw_fcs.a' % bld.env.FCS_TARGET)
    tsk = gprbuild_fcs(env=bld.env)
    tsk.set_outputs([lib])
    bld.add_to_group(tsk)
    bld.ada_fcs_lib = lib


@feature('cxxprogram')
@after_method('propagate_uselib_vars')
def ada_fcs_link(self):
    lib = getattr(self.bld, 'ada_fcs_lib', None)
    if lib is None or not getattr(self, 'link_task', None):
        return
    # after the vehicle libraries, which reference it
    self.env.append_value('STLIBPATH', [lib.parent.abspath()])
    self.env.append_value('STLIB', ['cw_fcs'])
    self.link_task.dep_nodes.append(lib)
