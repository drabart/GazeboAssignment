# GazeboAssignment

## Setup environment

- create python venv
- install `requirements.txt`
- add environment variables
    ```bash
    # .bashrc
    # Obtained at https://dev-portal.onshape.com/keys
    export ONSHAPE_API=https://cad.onshape.com
    export ONSHAPE_ACCESS_KEY=Your_Access_Key
    export ONSHAPE_SECRET_KEY=Your_Secret_Key
    ```

## Run simulation

In order to update the model do
`onshape-to-robot model`

Then to run it in gazebo run
`gz sim model/robot_world.sdf` 

