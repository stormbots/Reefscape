package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Elevator.GraphCommand.GraphCommandNode;



public class ElevatorStateManager {
    Elevator elevator;
    GraphCommand graph = new GraphCommand();

    GraphCommandNode boot;
    GraphCommandNode stowed;
    GraphCommandNode stowedUp;
    GraphCommandNode prepareToFloorPickUp;
    GraphCommandNode prepareToSourcePickup;
    GraphCommandNode sourcePickup;
    GraphCommandNode floorPickup;
    GraphCommandNode moveToL1Algae;
    GraphCommandNode moveToL2Algae;
    GraphCommandNode intakeAlgae;
    GraphCommandNode moveToL1;
    GraphCommandNode scoreToL1;
    GraphCommandNode moveToL2;
    GraphCommandNode scoreToL2;
    GraphCommandNode moveToL3;
    GraphCommandNode scoreToL3;
    GraphCommandNode moveToL4;
    GraphCommandNode scoreToL4;
    GraphCommandNode climbing;

    ElevatorStateManager(Elevator elevator){
        this.elevator = elevator;

        //addallthestates
        addStates();

        graph.setGraphRootNode(boot);
        graph.setTargetNode(stowed);

        //load process
        stowed.AddNode(stowedUp, 1);
        stowedUp.AddNode(prepareToSourcePickup,1);
        prepareToSourcePickup.AddNode(sourcePickup,1);

        stowedUp.AddNode(prepareToSourcePickup, 1);
        prepareToFloorPickUp.AddNode(sourcePickup, 1);
        stowedUp.AddNode(moveToL1Algae, 1);
        moveToL1Algae.AddNode(intakeAlgae, 1);
        stowedUp.AddNode(moveToL2Algae, 1);
        moveToL2Algae.AddNode(intakeAlgae, 1);
        stowedUp.AddNode(moveToL1, 1);
        moveToL1.AddNode(scoreToL1, 1);
        stowedUp.AddNode(moveToL2, 1);
        moveToL2.AddNode(scoreToL2, 1);
        stowedUp.AddNode(moveToL3, 1);
        moveToL3.AddNode(scoreToL3, 1);
        stowedUp.AddNode(moveToL4, 1);
        moveToL4.AddNode(scoreToL4, 1);       
        stowedUp.AddNode(climbing,1); 
    }

 

    private void addStates(){
        //createallthestates
        boot = graph.new GraphCommandNode("Boot",
        elevator.moveToPose(elevator.kStowed),
        elevator.moveToPose(elevator.kStowed),
        elevator.moveToPose(elevator.kStowed));

        stowed = graph.new GraphCommandNode("Stowed",
        elevator.moveToPose(elevator.kStowed),
        elevator.moveToPose(elevator.kStowed),
        elevator.moveToPose(elevator.kStowed));

        //TODO NEEDS TO EXIST PROPERLY
        stowedUp = graph.new GraphCommandNode("StowedUp",
        elevator.moveToPose(elevator.kStowedUp),
        elevator.moveToPose(elevator.kStowedUp),
        elevator.moveToPose(elevator.kStowedUp));

        //TODO NEEDS TO EXIST PROPERLY
        prepareToSourcePickup = graph.new GraphCommandNode("prepareToSourePickUp", 
        elevator.moveToPose(elevator.kMoveToSurcePickUp), 
        elevator.moveToPose(elevator.kMoveToSurcePickUp), 
        elevator.moveToPose(elevator.kMoveToSurcePickUp));

        sourcePickup = graph.new GraphCommandNode("SourcePickup",
        elevator.moveToPose(elevator.kStationPickup),
        elevator.moveToPose(elevator.kStationPickup),
        elevator.moveToPose(elevator.kStationPickup));

        //TODO NEEDS TO EXIST PROPERLY
        prepareToFloorPickUp = graph.new GraphCommandNode("prepareToFloorPickUp", 
        elevator.moveToPose(elevator.kMoveToFloorPickUp), 
        elevator.moveToPose(elevator.kMoveToFloorPickUp), 
        elevator.moveToPose(elevator.kMoveToFloorPickUp));

        floorPickup = graph.new GraphCommandNode("FloorPickup",
        elevator.moveToPose(elevator.kFloorPickup),
        elevator.moveToPose(elevator.kFloorPickup),
        elevator.moveToPose(elevator.kFloorPickup));

        //TODO NEEDS TO EXIST PROPERLY
        moveToL1Algae = graph.new GraphCommandNode("moveToL1Algae",
        elevator.moveToPose(elevator.moveToL1Algae),
        elevator.moveToPose(elevator.moveToL1Algae),
        elevator.moveToPose(elevator.moveToL1Algae));

        //TODO NEEDS TO EXIST PROPERLY
        moveToL2Algae = graph.new GraphCommandNode("moveToL2Algae",
        elevator.moveToPose(elevator.moveToL2Algae),
        elevator.moveToPose(elevator.moveToL2Algae),
        elevator.moveToPose(elevator.moveToL2Algae));

        //TODO NEEDS TO EXIST PROPERLY
        intakeAlgae = graph.new GraphCommandNode("moveToL2Algae",
        elevator.moveToPose(elevator.intakeAlgae),
        elevator.moveToPose(elevator.intakeAlgae),
        elevator.moveToPose(elevator.intakeAlgae));

        moveToL1 = graph.new GraphCommandNode("moveToL1", 
        elevator.moveToPose(elevator.kL1), 
        elevator.moveToPose(elevator.kL1), 
        elevator.scoreAtPose(elevator.kL1));

        scoreToL1 = graph.new GraphCommandNode("scoreToL1",
        elevator.moveToPoseWithScorer(elevator.kL1), 
        elevator.moveToPoseWithScorer(elevator.kL1), 
        elevator.moveToPoseWithScorer(elevator.kL1));

        moveToL2 = graph.new GraphCommandNode("moveToL2",
        elevator.moveToPose(elevator.kL2),
        elevator.moveToPose(elevator.kL2), 
        elevator.scoreAtPose(elevator.kL2));

        scoreToL2 = graph.new GraphCommandNode("scoreToL2",
        elevator.moveToPoseWithScorer(elevator.kL2), 
        elevator.moveToPoseWithScorer(elevator.kL2), 
        elevator.moveToPoseWithScorer(elevator.kL2));

        moveToL3 = graph.new GraphCommandNode("moveToL3",
        elevator.moveToPose(elevator.kL3),
        elevator.moveToPose(elevator.kL3), 
        elevator.scoreAtPose(elevator.kL3));

        scoreToL3 = graph.new GraphCommandNode("scoreToL3",
        elevator.moveToPoseWithScorer(elevator.kL3), 
        elevator.moveToPoseWithScorer(elevator.kL3), 
        elevator.moveToPoseWithScorer(elevator.kL3));

        moveToL4 = graph.new GraphCommandNode("moveToL4",
        elevator.moveToPose(elevator.kL4),
        elevator.moveToPose(elevator.kL4), 
        elevator.scoreAtPose(elevator.kL4));

        scoreToL4 = graph.new GraphCommandNode("scoreToL4",
        elevator.moveToPoseWithScorer(elevator.kL4), 
        elevator.moveToPoseWithScorer(elevator.kL4), 
        elevator.moveToPoseWithScorer(elevator.kL4));


        climbing = graph.new GraphCommandNode("Climbing",
        elevator.moveToPose(elevator.kClimbing),
        elevator.moveToPose(elevator.kClimbing),
        elevator.moveToPose(elevator.kClimbing));
    }
    

    // private void referenceExternalCode(){
    //     graph.setTargetNode(node);
    // }



}