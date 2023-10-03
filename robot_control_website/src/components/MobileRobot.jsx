// MobileRobot.js
import React, {useEffect, useState} from 'react';
import '../css/MobileRobot.css';
import {CameraFeed, UpdateVelocities} from './CameraFeed';
import ps4ControllerIcon from '../imgs/ps4controller.png';  

/**
 * In react, properties are called "props"
 */
const UpdateInfo = (props) => {
    return (
        <div className="column">
            <div className="update-header-font">
                {props.header} 
            </div>
            <div className="update-value-font">
                {props.value}
            </div>
        </div>
    );
}

const MobileRobot = () => {
  const [imgSrc, setImg] = useState(null);
  useEffect(() => {
    const imgRawData = CameraFeed();
    setImg(imgRawData);
    }, []);

    const [spacePressed, setSpacePressed] = useState(false);
    useEffect(() => {
        const keydownHandler = (event) => {
            if (event.code === 'Space'){
                setSpacePressed(spacePressed => !spacePressed);
            }
        }
        window.addEventListener('keydown', keydownHandler);
        return () => {
            // prevent memory leak when the component gets unmounted
            window.removeEventListener('keydown', keydownHandler);
        }
    }, []);

  return (
    <div className="mobile-robot-container">
      <div className="image-window">
        {imgSrc && <img src={imgSrc} alt="mobile robot view" className='image'/>}
      </div>
      <div className="control-panel">
        <div className="row">
          <UpdateInfo header="Left Wheel Velocity: " value="10 m/s"/>
          <UpdateInfo header="Right Wheel Velocity: " value="10 m/s"/>
        </div>
        <div className="row">
          <UpdateInfo header="Linear Velocity: " value="20 m/s"/>
          <UpdateInfo header="Angular Velocity: " value="3 rad/s"/>
        </div>
        <div className="row">
          <div className="column icon-column">
            <div className={spacePressed? 'green-icon': 'black-icon'}>
                <img src={ps4ControllerIcon} alt="PS4 Controller" height="30px" />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export {MobileRobot};
