import React, { useState, useEffect } from 'react';
import LaughEmoji from '../imgs/laugh.jpg'
/**
 * Notes:
 * - In react, functions returning JSX are considered "functional components".
 *     - That's component based design of react
 *     - All caps to distinguish from HTML elements
 * - useState(initialState): returns a state variable and a function to update it
 *  - imports & exports: you can export a module as default. Then you can name it anything you want
 *  - require() comes from node, import comes from ES6
 * - useEffect will be called when rendering the page. it adds a new hook
 *  - your setFunction can take in a function like: state => new_state
 * - reader:
 *  - onload(): called when read operation is done
 *  - readAsDataURL(): data URL is the file data represented as a Base64 String
 *  - DataURL can be rendered by the browser in <img> 
 *  - {imgSrc && <img src={imgSrc} alt='Selected' height="200px"/>}. imgSrc is to see if url is not null
 */

export function CameraFeed() {
    return LaughEmoji;
};

export function UpdateVelocities(){
  return (
    <div>
      <div>Velocity: {2} km/h</div>
    </div>
  );
}

