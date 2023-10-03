import './App.css';
import './index.css';
import {MobileRobot} from './components/MobileRobot';

/**
 * Main function that returns all JSX to index.js
 */
function App() {
  return (
    <div className="App">
        <div className="container">
            <h1 className="title">RJJJ Control Center</h1>
            <MobileRobot />
        </div>
    </div>
  );
}

export default App;
