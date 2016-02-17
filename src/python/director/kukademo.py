from PythonQt import QtCore, QtGui, QtUiTools

from director import transformUtils

import director.applogic as app
import director.objectmodel as om
from director.asynctaskqueue import AsyncTaskQueue
from director import objectmodel as om
from director import visualization as vis
from director import robotstate
from director import planplayback
from director import vtkAll as vtk
from director.simpletimer import SimpleTimer
from director import affordanceupdater

from director.debugVis import DebugData
from director import affordanceitems
from director import ikplanner

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


def clearLayout(w):
    children = w.findChildren(QtGui.QWidget)
    for child in children:
        child.delete()


class KukaDemo(object):

    def __init__(self, robotStateModel, ikPlanner, manipPlanner, sensorJointController, planPlaybackFunction):
        assert ikPlanner.fixedBaseArm == True
        
        self.planPlaybackFunction = planPlaybackFunction
        self.robotStateModel = robotStateModel
        self.manipPlanner = manipPlanner
        self.ikPlanner = ikPlanner
        self.sensorJointController = sensorJointController

        self.plans = []

        # top level switch between BDI or IHMC (locked base) and MIT (moving base and back)
        self.lockBack = True
        self.lockBase = True
        self.graspingHand = 'left'

        self.constraintSet = []

        self.targetSweepType = 'orientation' # gaze or orientation - but i've had problems with the gaze constraint
        self.coneThresholdDegrees = 5.0 # 0 is ok for reaching but often too tight for a trajectory
        self.boxLength = 0.3

    def addPlan(self, plan):
        self.plans.append(plan)

    def planPostureFromDatabase(self, groupName, postureName, side='left'):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, groupName, postureName, side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    ######### Higher Level Planning Functions ##################################################################

    def planRoomReach(self):
        # A single one shot gaze-constrained reach: place xyz at goal and align y-axis of hand with x-axis of goal
        self.initConstraintSet()
        self.addConstraintForTargetFrame(self.startFrame, 1)
        self.planTrajectory()

    ######### Lower Level Planning Functions ##################################################################
    def planTrajectory(self):
        self.ikPlanner.ikServer.usePointwise = False
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)

    def initConstraintSet(self):
        # create constraint set
        startPose = self.getPlanningStartPose()
        startPoseName = 'gaze_plan_start'
        endPoseName = 'gaze_plan_end'
        self.ikPlanner.addPose(startPose, startPoseName)
        self.ikPlanner.addPose(startPose, endPoseName)
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, [], startPoseName, endPoseName)
        self.constraintSet.endPose = startPose

        # add body constraints
        bodyConstraints = self.ikPlanner.createMovingBodyConstraints(startPoseName, lockBase=self.lockBase, lockBack=self.lockBack, lockLeftArm=self.graspingHand=='right', lockRightArm=self.graspingHand=='left')
        self.constraintSet.constraints.extend(bodyConstraints)

    def addConstraintForTargetFrame(self,goalFrame, t):
        if (self.targetSweepType is 'orientation'):
            self.appendPositionOrientationConstraintForTargetFrame(goalFrame, t)
        elif (self.targetSweepType is 'gaze'):
            # align the palmGazeAxis axis (on the hand) with the vector 'targetAxis' from worldToTargetFrame?
            palmGazeAxis = self.ikPlanner.getPalmToHandLink(self.graspingHand).TransformVector([0,1,0])
            self.appendPositionGazeConstraintForTargetFrame(goalFrame, t, targetAxis=[0.0, 0.0, 1.0], bodyAxis=palmGazeAxis)

    def appendPositionGazeConstraintForTargetFrame(self, goalFrame, t, targetAxis=[-1.0, 0.0, 0.0], bodyAxis=[-1.0, 0.0, 0.0]):
        gazeConstraint = self.ikPlanner.createGazeGraspConstraint(self.graspingHand, goalFrame, self.graspToHandLinkFrame, self.coneThresholdDegrees , targetAxis, bodyAxis)
        gazeConstraint.tspan = [t, t]
        self.constraintSet.constraints.insert(0, gazeConstraint)

        positionConstraint, _ = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)

    def appendPositionOrientationConstraintForTargetFrame(self, goalFrame, t):
        positionConstraint, orientationConstraint = self.ikPlanner.createPositionOrientationGraspConstraints(self.graspingHand, goalFrame, self.graspToHandLinkFrame)
        positionConstraint.tspan = [t, t]
        orientationConstraint.tspan = [t, t]
        self.constraintSet.constraints.append(positionConstraint)
        self.constraintSet.constraints.append(orientationConstraint)


    ### End Planning Functions ####################################################################
    ########## Glue Functions #####################################################################
    def printAsync(self, s):
        yield
        print s

    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

    def playSequenceNominal(self):
        assert None not in self.plans
        self.planPlaybackFunction(self.plans)

    def commitManipPlan(self):
            self.manipPlanner.commitManipPlan(self.plans[-1])

    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        return self.delay(planElapsedTime + 1.0)

    def animateLastPlan(self):
        plan = self.plans[-1]
        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)

    def doneIndicator(self):
        print "We are done here."

    # def autonomousRoomMapNew(self, side='left'):
    #     taskQueue = AsyncTaskQueue()
    #     lowSpeed = 5
    #     highSpeed = 30
    #     delayTime = 3 # TODO: for potential self.delay to wait for pointclouds to be registered

    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'General', 'arm up pregrasp'))
    #     taskQueue.addTask(self.animateLastPlan)

    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p1_up'))
    #     taskQueue.addTask(self.animateLastPlan)
    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p1_down', side=side))
    #     taskQueue.addTask(self.animateLastPlan)

    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p2_down', side=side))
    #     taskQueue.addTask(self.animateLastPlan)
    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p2_up', side=side))
    #     taskQueue.addTask(self.animateLastPlan)

    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p3_up', side=side))
    #     taskQueue.addTask(self.animateLastPlan)
    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p3_down', side=side))
    #     taskQueue.addTask(self.animateLastPlan)

    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p4_down', side=side))
    #     taskQueue.addTask(self.animateLastPlan)
    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p4_up', side=side))
    #     taskQueue.addTask(self.animateLastPlan)

    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p5_up', side=side))
    #     taskQueue.addTask(self.animateLastPlan)
    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, lowSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'roomMapping', 'p5_down', side=side))
    #     taskQueue.addTask(self.animateLastPlan)

    #     taskQueue.addTask(functools.partial(self.setMaxDegreesPerSecond, highSpeed))
    #     taskQueue.addTask(functools.partial(self.planPostureFromDatabase, 'General', 'arm up pregrasp', side=side))
    #     taskQueue.addTask(self.animateLastPlan)

    #     taskQueue.addTask(self.doneIndicator)
    #     return taskQueue

class KukaDemoPanel(object):

    def __init__(self): #, robotStateModel, robotStateJointController, teleopRobotModel, ikPlanner, manipPlanner,
        # showPlanFunction, hidePlanFunction):

        # self.robotStateModel = robotStateModel
        # self.robotStateJointController = robotStateJointController
        # self.ikPlanner = ikPlanner
        # self.manipPlanner = manipPlanner
        # self.showPlanFunction = showPlanFunction
        # self.hidePlanFunction = hidePlanFunction
        #
        # manipPlanner.connectPlanCommitted(self.onPlanCommitted)

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(':/ui/ddKukaDemoPanel.ui')
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        uifile.close()

        self.ui = WidgetDict(self.widget.children())
        #self.ui.postureDatabaseButton.connect('clicked()', self.onPostureDatabaseClicked)

    # def disableJointTeleop(self):
    #     self.ui.jointTeleopFrame.setEnabled(False)
    #
    # def disableEndEffectorTeleop(self):
    #     self.ui.endEffectorTeleopFrame.setEnabled(False)
    #
    # def jointTeleopActivated(self):
    #     self.disableEndEffectorTeleop()
    #
    # def endEffectorTeleopActivated(self):
    #     self.disableJointTeleop()
    #
    # def endEffectorTeleopDeactivated(self):
    #     self.hideTeleopModel()
    #     self.enablePanels()
    #
    # def jointTeleopDeactivated(self):
    #     self.hideTeleopModel()
    #     self.enablePanels()
    #
    # def enablePanels(self):
    #     self.ui.endEffectorTeleopFrame.setEnabled(True)
    #     self.ui.jointTeleopFrame.setEnabled(True)
    #
    # def onPlanCommitted(self, plan):
    #     self.hideTeleopModel()
    #
    # def hideTeleopModel(self):
    #     self.teleopRobotModel.setProperty('Visible', False)
    #     self.robotStateModel.setProperty('Visible', True)
    #     self.robotStateModel.setProperty('Alpha', 1.0)
    #
    # def showTeleopModel(self):
    #     self.teleopRobotModel.setProperty('Visible', True)
    #     self.robotStateModel.setProperty('Visible', True)
    #     self.robotStateModel.setProperty('Alpha', 0.1)
    #
    # def showPose(self, pose):
    #     self.teleopJointController.setPose('teleop_pose', pose)
    #     self.hidePlanFunction()
    #     self.showTeleopModel()
    #
    # def showPlan(self, plan):
    #     self.hideTeleopModel()
    #     self.showPlanFunction(plan)

def _getAction():
    return app.getToolBarActions()['ActionMappingPanel']

def init(): #robotStateModel, robotStateJointController, manipPlanner, showPlanFunction, hidePlanFunction):

    global panel
    global dock

    panel = KukaDemoPanel()#robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController, debrisPlanner, manipPlanner, affordanceManager, showPlanFunction, hidePlanFunction)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
#
# '''
# Mapping Image Fit for live-stream of webcam
# '''
# class MappingImageFitter(ImageBasedAffordanceFit):
#
#     def __init__(self, mappingDemo):
#         ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
#         self.mappingDemo = mappingDemo
#
#     def fit(self, polyData, points):
#         pass
#
# '''
# Kuka Demo Task Panel
# '''
# class KukaDemoTaskPanel(TaskUserPanel):
#
#     def __init__(self, kukaDemo):
#
#         TaskUserPanel.__init__(self, windowTitle='Kuka Demo')
#
#         self.kukaDemo = kukaDemo
#
#         #self.addDefaultProperties()
#         self.addButtons()
#         self.addTasks()
#
#         self.fitter = MappingImageFitter(self.mappingDemo)
#         self.initImageView(self.fitter.imageView)
#
#     def addButtons(self):
#         self.addManualSpacer()
#         self.addManualButton('Low Stiffness', self.mappingPanel.onStartMappingButton)
#         self.addManualSpacer()
#         self.addManualButton('High Stiffness', self.mappingPanel.onStopMappingButton)
#
#     def addDefaultProperties(self):
#         # Properties to include: radius, z_offset, x distance, stiffness
#         '''self.params.addProperty('Hand', 0,
#                                 attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
#         self.params.addProperty('Base', 0,
#                                 attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
#         if self.tableDemo.fixedBaseArm:
#             self.params.addProperty('Back', 0,
#                                     attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
#         else:
#             self.params.addProperty('Back', 1,
#                                     attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
#         self._syncProperties()'''
#
#     def onPropertyChanged(self, propertySet, propertyName):
#         self.taskTree.removeAllTasks()
#         self.addTasks()
#
#     def addTasks(self):
#
#         # some helpers
#         def addTask(task, parent=None):
#             self.taskTree.onAddTask(task, copy=False, parent=parent)
#
#         def addFunc(func, name, parent=None, confirm=False):
#             addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
#             if confirm:
#                 addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=parent)
#
#         def addManipulation(func, name, parent=None, confirm=False):
#             group = self.taskTree.addGroup(name, parent=parent)
#             addFunc(func, name='plan motion', parent=group)
#             addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
#             addFunc(v.commitManipPlan, name='execute manip plan', parent=group)
#             addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'),
#                     parent=group)
#             if confirm:
#                 addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'),
#                         parent=group)
#
#         v = self.kukaDemo
#
#         self.taskTree.removeAllTasks()
#
#         # graspingHand is 'left', side is 'Left'
#         side = 'left' #self.params.getPropertyEnumValue('Hand')
#
#         ###############
#         if v.ikPlanner.fixedBaseArm:
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'General', 'arm up pregrasp', side=side), 'Arm up')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p1_up', side=side), 'Pose 1')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p1_down', side=side), 'Pose 2')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p2_up', side=side), 'Pose 3')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p2_down', side=side), 'Pose 4')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p3_up', side=side), 'Pose 5')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p3_down', side=side), 'Pose 6')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p4_up', side=side), 'Pose 7')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p4_down', side=side), 'Pose 8')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p5_up', side=side), 'Pose 9')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'roomMapping', 'p5_down', side=side), 'Pose 10')
#             addManipulation(functools.partial(v.planPostureFromDatabase, 'General', 'arm up pregrasp', side=side), 'Arm up')
