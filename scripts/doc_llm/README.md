# Guide to Generate ROS Node README Files with GitHub Copilot Edits

This guide explains how to use GitHub Copilot Edits to automatically generate README.md files for ROS nodes based on Python source files.

## Required Files

To generate documentation properly, make sure you have:
- Python (`.py`) files containing your ROS nodes
- A template file named `node_README_example.md` 
- A `README.md` file to be updated (or create a new file if needed)
- YAML file(s) (e.g., `detection.yaml`) containing parameter defaults

## VS Code Instructions

1. Open your project in VS Code

2. Open GitHub Copilot Edits by clicking on the Copilot icon in the sidebar or pressing `Ctrl+Shift+I` (Windows/Linux) or `Cmd+Shift+I` (Mac)

3. In the working files section, make sure to add the required files. You can do this by:
   - Opening each file and keeping them in editor tabs, or
   - Adding files to the working set (Right-click the editor tab and select "Add to Working Set" or press `Ctrl+Shift+W`)

4. Enter one of the following queries in the Copilot Edits:

   For creating a new README file:
   ```
   Using node_README_example.md as a template, update README.md file, generating documentation for ROS nodes in the provided .py files.
   
   Obtain parameter default values from .yaml file(s). Put the parameters without default values on the top of the table and assign '-'.
   
   For the subscribed and published topics names obtain names from code verbatim.
   ```

   For updating an existing README file:
   ```
   Update README.md file, updating documentation for ROS nodes and only for ROS nodes in the provided .py files. Don't change documentation for nodes that are not named in the provided files.

   Obtain parameter default values from .yaml file(s). Put the parameters without default values on the top of the table and assign '-'.

   For the subscribed and published topics names obtain names from code verbatim.   
   ```

5. Review the generated content and click "Insert at Cursor" to add it to your currently open file, or choose "Create New File" to create a new README.md

## JetBrains IDEs (PyCharm) Instructions

1. Open your project in PyCharm

2. Open GitHub Copilot by clicking the Copilot icon in the bottom toolbar or using `Alt+C`

3. Add the required files to GitHub Copilot Edits working set 

4. Enter one of the following queries in the Copilot Edits:

   For creating a new README file:
   ```
   Using node_README_example.md as a template, update README.md file, generating documentation for ROS nodes in the provided .py files.
   
   Obtain parameter default values from .yaml file(s). Put the parameters without default values on the top of the table and assign '-'.
   
   For the subscribed and published topics names obtain names from code verbatim.
   ```

   For updating an existing README file:
   ```
   Update README.md file, updating documentation for ROS nodes and only for ROS nodes in the provided .py files. Don't change documentation for nodes that are not named in the provided files.

   Obtain parameter default values from .yaml file(s). Put the parameters without default values on the top of the table and assign '-'.

   For the subscribed and published topics names obtain names from code verbatim.
   ```

5. Review the generated content and apply the suggested edits to your README.md file
