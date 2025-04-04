## HowTo:

- Use this Project as Template for your Repository: [HowTo Guide](https://gitlab.informatik.uni-bremen.de/triple/gnc/scenario_simulation/-/wikis/set-up-repo-from-template)

- Adapt the pathes of the Badges
  - Go to `Settings->General->Bagdes` Click on `Expand`
  - Edit the Link of 
    - `Test Coverage`
      - Replace `gnc/misc/cmake_packag` with the path to your repository
    - `Documentation Coverage`
      - Replace `gnc/misc/cmake_packag` with the path to your repository
    - `Documentation`
      - Replace `gnc/misc/cmake_packag` with the path to your repository
    
- Give your repository access to the gitlab ci templates.
  - Open: https://gitlab.informatik.uni-bremen.de/triple/gnc/misc/ci-config
  - Go to: `Settings->CI/CD->Token Access`
  - Add your Project at `Allow CI job tokens from the following projects to access this project`
- write lots of unit tests :-)

## General:

Coding Guidelines:
- see http://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html

## Reports:

Coverage Report:
- https://triple.glpages.informatik.uni-bremen.de/gnc/misc/cmake_package/fancy_html_coverage_report/

Dokumentation: (Doxygen Awesome)
- https://triple.glpages.informatik.uni-bremen.de/gnc/misc/cmake_package/doxygen_awesome/html/

Documentation Coverage:
- https://triple.glpages.informatik.uni-bremen.de/gnc/misc/cmake_package/doxygen_awesome/doc-coverage/

Code Climate Report:
- https://triple.glpages.informatik.uni-bremen.de/gnc/misc/cmake_package/gl-code-quality-report.html


## Legacy

Documentation (Doxygen Rosdoc Lite)
- https://triple.glpages.informatik.uni-bremen.de/gnc/misc/cmake_package/rosdoc_lite/html/
