# Support for dummy displays (keys only)
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class none:
    def __init__(self, config):
        pass
    def build_config(self):
        pass
    def send(self, cmds, is_data=False, is_extended=False):
        pass
    def flush(self):
        pass
    def init(self):
        pass
    def cache_glyph(self, glyph_name, base_glyph_name, glyph_id):
        pass
    def write_text(self, x, y, data):
        pass
    def write_graphics(self, x, y, row, data):
        pass
    def set_glyphs(self, glyphs):
        pass
    def write_glyph(self, x, y, glyph_name):
        return 1
    def clear(self):
        pass
    def get_dimensions(self):
        return (16, 4)
