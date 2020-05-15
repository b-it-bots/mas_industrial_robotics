# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import subprocess
# sys.path.insert(0, os.path.abspath('.'))

current_file_dir = os.path.dirname(os.path.realpath(__file__))

def get_git_short_hash():
    rc = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'])
    rc = rc.decode("utf-8").strip()
    return rc



# -- Project information -----------------------------------------------------
project = 'MAS Industrial Robotics'
copyright = '2019, www.b-it-bots.de'
author = 'www.b-it-bots.de'

# The full version, including alpha/beta/rc tags
#current_hash = get_git_short_hash()
current_hash = 'Kinetic'
version = "latest ({})".format(current_hash)
release = version


# -- General configuration ---------------------------------------------------
# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"
theme_path = os.path.join(current_file_dir, "..", "..", "third_party",
                          "sphinx_rtd_theme")
html_theme_path = [theme_path, ]

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = []

# Master document should be index.rst instead of contents.rst
# https://github.com/readthedocs/readthedocs.org/issues/2569#issuecomment-485117471
master_doc = 'index'
