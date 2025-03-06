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
    GraphCommandNode intakeAlgaeL1;
    GraphCommandNode intakeAlgaeL2;
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
        moveToL1Algae.AddNode(intakeAlgaeL1, 1);
        stowedUp.AddNode(moveToL2Algae, 1);
        moveToL2Algae.AddNode(intakeAlgaeL2, 1);
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

    /////////////////////////////////////////////////////////////////////////////////////////////////
    //TODO Most States need to exsist properly, states might need to have Scoring motor implemented//
    /////////////////////////////////////////////////////////////////////////////////////////////////

    private void addStates(){
        boot = graph.new GraphCommandNode("Boot",
        elevator.moveToPoseSafe(elevator.kStowed),
        elevator.moveToPoseSafe(elevator.kStowed),
        elevator.moveToPoseSafe(elevator.kStowed));

        stowed = graph.new GraphCommandNode("Stowed",
        elevator.moveToPoseSafe(elevator.kStowed),
        elevator.moveToPoseSafe(elevator.kStowed),
        elevator.moveToPoseSafe(elevator.kStowed));

        stowedUp = graph.new GraphCommandNode("StowedUp",
        elevator.moveToPoseSafe(elevator.kStowedUp),
        elevator.moveToPoseSafe(elevator.kStowedUp),
        elevator.moveToPoseSafe(elevator.kStowedUp));

        //TODO Needs Actual pose
        prepareToSourcePickup = graph.new GraphCommandNode("prepareToSourePickUp", 
        elevator.moveToPoseSafe(elevator.kStationPickup), 
        elevator.moveToPoseSafe(elevator.kStationPickup), 
        elevator.moveToPoseSafe(elevator.kStationPickup));

        sourcePickup = graph.new GraphCommandNode("SourcePickup",
        elevator.moveToPoseSafe(elevator.kStationPickup),
        elevator.moveToPoseSafe(elevator.kStationPickup),
        elevator.moveToPoseSafe(elevator.kStationPickup));

        // Floor Intake is gone ):
        // prepareToFloorPickUp = graph.new GraphCommandNode("prepareToFloorPickUp", 
        // elevator.moveToPoseSafe(elevator.kFloorPickup), 
        // elevator.moveToPoseSafe(elevator.kFloorPickup), 
        // elevator.moveToPoseSafe(elevator.kFloorPickup));

        // FloornIntake is gone ):
        // floorPickup = graph.new GraphCommandNode("FloorPickup",
        // elevator.moveToPoseSafe(elevator.kFloorPickup),
        // elevator.moveToPoseSafe(elevator.kFloorPickup),
        // elevator.moveToPoseSafe(elevator.kFloorPickup));


        //TODO Needs Actual pose
        intakeAlgaeL1 = graph.new GraphCommandNode("prepareToPickUpL2Algae",
        elevator.moveToPoseSafe(elevator.kL2Algae),
        elevator.moveToPoseSafe(elevator.kL2Algae),
        elevator.moveToPoseSafe(elevator.kL2Algae));

        intakeAlgaeL2 = graph.new GraphCommandNode("pickUpL2Algae",
        elevator.moveToPoseSafe(elevator.kL2Algae),
        elevator.moveToPoseSafe(elevator.kL2Algae),
        elevator.moveToPoseSafe(elevator.kL2Algae));

        moveToL1Algae = graph.new GraphCommandNode("prepareToPickUpL3Algae",
        elevator.moveToPoseSafe(elevator.kL3Algae),
        elevator.moveToPoseSafe(elevator.kL3Algae),
        elevator.moveToPoseSafe(elevator.kL3Algae));

        moveToL1 = graph.new GraphCommandNode("moveToL1", 
        elevator.moveToPoseSafe(elevator.kL1), 
        elevator.moveToPoseSafe(elevator.kL1),
        elevator.moveToPoseSafe(elevator.kL1)
        //elevator.scoreAtPose(elevator.kL1)
        );

        scoreToL1 = graph.new GraphCommandNode("scoreToL1",
        elevator.moveToPoseSafe(elevator.kL1), 
        elevator.moveToPoseSafe(elevator.kL1), 
        elevator.moveToPoseSafe(elevator.kL1)
        );

        moveToL2 = graph.new GraphCommandNode("moveToL2",
        elevator.moveToPoseSafe(elevator.kL2),
        elevator.moveToPoseSafe(elevator.kL2),
        elevator.moveToPoseSafe(elevator.kL2) 
        );

        scoreToL2 = graph.new GraphCommandNode("scoreToL2",
        elevator.moveToPoseSafe(elevator.kL2), 
        elevator.moveToPoseSafe(elevator.kL2), 
        elevator.moveToPoseSafe(elevator.kL2)
        //elevator.scoreAtPose(elevator.kL2)
        //Graph command nodes need 3 poses, score at pose needs tobe implemented know that coral scorer is gone
        );

        moveToL3 = graph.new GraphCommandNode("moveToL3",
        elevator.moveToPoseSafe(elevator.kL3),
        elevator.moveToPoseSafe(elevator.kL3), 
        elevator.moveToPoseSafe(elevator.kL3));

        scoreToL3 = graph.new GraphCommandNode("scoreToL3",
        elevator.moveToPoseSafe(elevator.kL3), 
        elevator.moveToPoseSafe(elevator.kL3), 
        elevator.moveToPoseSafe(elevator.kL3)
        //elevator.scoreAtPose(elevator.kL3)
        //Graph command nodes need 3 poses, score at pose needs tobe implemented know that coral scorer is gone
        );

        moveToL4 = graph.new GraphCommandNode("moveToL4",
        elevator.moveToPoseSafe(elevator.kL4),
        elevator.moveToPoseSafe(elevator.kL4), 
        elevator.moveToPoseSafe(elevator.kL4));

        scoreToL4 = graph.new GraphCommandNode("scoreToL4",
        elevator.moveToPoseSafe(elevator.kL4), 
        elevator.moveToPoseSafe(elevator.kL4), 
        elevator.moveToPoseSafe(elevator.kL4)
        //elevator.scoreAtPose(elevator.kL4)
        //Graph command nodes need 3 poses, score at pose needs tobe implemented know that coral scorer is gone
        );


        climbing = graph.new GraphCommandNode("Climbing",
        elevator.moveToPoseSafe(elevator.kClimbing),
        elevator.moveToPoseSafe(elevator.kClimbing),
        elevator.moveToPoseSafe(elevator.kClimbing));
    }
    

    // private void referenceExternalCode(){
    //     graph.setTargetNode(node);
    // }



}