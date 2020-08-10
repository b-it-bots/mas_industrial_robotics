.. _coding_style:

Coding guidelines
=================

.. note::

  Make sure that your text editor is properly configured to use **spaces** instead of tabs.
  All C/C++ code should be written and formatted according to the **Google style guide** (with an exception to column limit
  and breaking braces.
  See `.clang-format <https://github.com/b-it-bots/mas_industrial_robotics/blob/kinetic/.clang-format>`_ for more details).
  All Python code should adhere to the **PEP-8 style guide**.

.. _style-label:

Coding style
------------

C/C++
^^^^^

* Linting

  `cpplint <https://github.com/cpplint/cpplint>`_ is used to lint C++ code according to the
  `Google style guide <https://google.github.io/styleguide/cppguide.html>`_.
  cpplint can be installed using pip (in a python 3.7 environment)

  .. code-block:: bash

    pip install cpplint

  Run cpplint on a file/directory as follows

  .. code-block:: bash

    cpplint <filename/directory>


* Code formatting

  `Clang format <https://clang.llvm.org/docs/ClangFormat.html>`_ is used to format C/C++ code.
  Install clang format as follows.

  .. code-block:: bash

    sudo apt-get install clang-format

  All configurations are present in .clang-format file and its is mandatory to use this file to format
  the code in this repository.
  Please note that it is necessary to run clang-format from the repository's root folder so that it
  uses the repository's .clang-format file. To run clang-format on a single C++ file, use

  .. code-block:: bash

    clang-format -i <C++ filename>

  To run clang-format on files inside a directory recursively, use

  .. code-block:: bash

    find . -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -i {} \;

* Static code analysis

  `Cppcheck <http://cppcheck.sourceforge.net/>`_ is used for static code analysis in order to detect
  bugs and undefined behaviors due to bad coding constructs. Install cppcheck using

  .. code-block:: bash

    sudo apt-get install cppcheck

  To run cppcheck on a file or directory, run

  .. code-block:: bash

    cppcheck <filename/directory>



Python
^^^^^^

* Linting

  Python code should follow the `PEP-8 <https://www.python.org/dev/peps/pep-0008/>`_ style guide.
  Installing linters in your python environment ensures compliance with the PEP-8 style guide.
  The precommit hooks for this repository uses pylint in a **python 3.7 environment** and can lint
  python 2 code. Pylint can
  be easily installed using pip.

  .. code-block:: bash

    pip install pylint

  In order to analyze file/s for linting errors, manually run pylint using the following command.

  .. code-block:: bash

    pylint <python filename or directory name>

* Sorting imports

  `isort <https://timothycrosley.github.io/isort/>`_ organizes and sorts imports in python files.
  Install isort using pip.

  .. code-block:: bash

    pip install isort

  To run isort on a python file use

  .. code-block:: bash

    isort <python filename>

* Code formatter

  `Black <https://black.readthedocs.io/en/stable/installation_and_usage.html>`_ is used to format python
  code. Please ensure that your code is formatted using black before committing your changes.
  Black can be installed using pip (again in a **python 3.7 environment**).

  .. code-block:: bash

    pip install black

  To format existing python code using black, run the following

  .. code-block:: bash

    black <python filename/directory>


.. note::

  `Pre-commit <https://pre-commit.com/#intro>`_ hooks has been added to this repository.
  Please note that you will not be able to locally commit your changes to git until all the
  checks in the .pre-commit-config.yaml pass. Some serious violations of the standard coding
  guidelines will result in errors while running git commit and have to be manually fixed.
  **Users will not be able to commit their code, until these errors are fixed.**
  Please ensure that **git commit** or **pre-commit hooks** (and not the code itself) is run in a **python 3.7 environment**
  as configured in .pre-commit-config.yaml.

.. warning::

  Alternatively, one could also verify if the pre-commit hooks pass before actually committing
  the code to git. To do so please run the following command after making necessary changes to
  your code.

  .. code-block:: bash

    pre-commit run --all-files

  This is however currently discouraged because there are several linting errors in the whole
  repository yet to be fixed and one doesn't want to end up fixing thousands of errors when
  just trying to add their contribution.


Editors for software development
--------------------------------

* Visual Studio Code
* Vim
* Pycharm

Install the necessary python, C++ and ROS plugins after installing a desired editor.
Other editors which support ROS are listed `here <http://wiki.ros.org/IDEs>`_.

Configuring editors
^^^^^^^^^^^^^^^^^^^

It is important to configure your editor settings so that linters, code formatters and
code checkers check for errors (and solve them if possible) automatically upon saving
your changes in a file.
Below is an illustration of the settings configurations that need to modified in Visual Studio Code to avoid
manually performing the checks described in :ref:`style-label`. Similar configurations can be done in
other editors too.

The settings can be configured through the Settings option in File menu or in
settings.json.

* Python linting
    By default pylint is enabled in Visual Studio Code, however pylint has to be installed
    using pip in your chosen python interpreter path. Please do not enable other linters as
    this could create a conflict while running pre-commit hooks. Please checkout the
    `VS Code website <https://code.visualstudio.com/docs/python/linting>`_ for more information.

* Python code formatting
    Since pre-commit hooks uses black to format python code, this can be very conveniently
    added to your editor so that the file is auto-formatted by black upon saving. Add the following
    to your settings.json to enable black code formatting.

    .. code-block:: bash

      "python.formatting.blackArgs": [
      "--line-length=79"
      ],
      "python.formatting.provider": "black",
      "[python]": {
      "editor.codeActionsOnSave": {
      "source.organizeImports": true
      }
      },

* C++ linting
    Install the cpplint extension to VS Code to enable the cpplinter. This then highlights the linting
    errors in the C++ code with squiggly lines.

* C++ code formatting
    Clang-format is used to format C++ code. This can be configured in settings.json
    after installing the official Microsoft C/C++ extension. Add the following lines to your
    settings.json file so that the configurations from .clang-format in the repository are
    used by VS Code to format the C++ files.

    .. code-block:: bash

      "C_Cpp.clang_format_style": "file",
      "C_Cpp.formatting": "clangFormat"


