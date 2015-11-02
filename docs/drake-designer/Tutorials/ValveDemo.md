# Valve Demo

## With Valkyrie

### Preparation
1. Start a roscore: ``roscore``
1. Start __bot-procman-sheriff__ with a local deputy and load the Valkyrie configuration file: ``bot-procman-sheriff -l $DRC_BASE/software/config/valkyrie.pmd``. Go to <kbd>Scripts</kbd> and click <kbd>start_ui</kbd>
1. Start Eclipse: ``eclipse`` with the imported workspace and Gradle project, then search for the ``ValkyrieROSAPISimulator`` by opening the Resource finder with <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>T</kbd> or going to <kbd>Project</kbd>--><kbd>Find Resource</kbd>
1. Start the ValkyrieROSAPISimulator by clicking on the menu next to the debug arrow and selecting "Start as Java Application"
1. Make sure that Valkyrie moves its hands inwards and goes down by slightly bending its kneeds both in drake-designer as well as in SCS
1. Hit the <kbd>F10</kbd> key in SCS and click on __Hide All__ to hide the force visualisation (otherwise it will collide with it)

### Inside drake-designer
1. Open the Task Panel by pressing <kbd>F10</kbd>
1. Navigate to the __Valve__ tab
1. Select Turning Mode: ``Human-like Turn`` from the options panel
