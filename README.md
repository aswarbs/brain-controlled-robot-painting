

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting">
    <img src="Assets/logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Brain Controlled Robot Painting</h3>

  <p align="center">
    Brain Controlled Robot Painting is a robotic installation designed to create paintings based on
    the participant's brain activity data.
    <br />
    <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting/-/wikis/home"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    ·
    <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting/-/issues">Report Bug</a>
    ·
    <a href="https://projects.cs.nott.ac.uk/psyas17/brain-controlled-robot-painting/-/issues">Request Feature</a>
  </p>
</div>




<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

  Brain Controlled Robot Painting is a robotic installation designed to create paintings based on
  the participant's brain activity data. The project uses the EEG Muse 2 band to collect the
  participant's brain activity data, which is then sent to a robotic arm (UR3e - Universal Robot Arm)
  that creates a painting based on the data. The installation was designed by an ambitious group of
  2nd year Computer Science students at the University of Nottingham.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple example steps.

### Prerequisites

- [Muse 2 Sensor](https://choosemuse.com/products/muse-s-gen-2-subscription/): This is the brain sensor we use to record the brainwaves for both the physical and digital versions of the program.
(Note: other brain sensors may be suitable, if there is an option to record the power frequency data in the form of a CSV.)

- [Universal Robot UR3e Arm](https://www.universal-robots.com/products/ur3-robot/?utm_source=Google&utm_medium=cpc&utm_cja=Demo&utm_leadsource=Paid%20Search&utm_campaign=HQ_UK_Always-On2021&utm_content=textad&utm_term=ur3e&gclid=CjwKCAjw0ZiiBhBKEiwA4PT9zzCeKHdzOQXcTTBz48I0TD7OVmmo0pPlBwlILHntiE7iao-VbE0PnhoCXwoQAvD_BwE): This is the arm we use to draw the paintings in the physical version of the program. 

- [Gripper attachment](https://projects.cs.nott.ac.uk/comp2002/2022-2023/team9_project/-/blob/main/Prototypes/CMS%20UR3e%20Pen%20Mount.stl) : We 3D printed a custom attachment to hold the pen, which is located in our repository under “Prototypes/CMS UR3e Pen Mount.stl”, along with rubber bands to secure the pen in place. If you don’t have a 3D printer, you can use one of Universal Robots’ advertised grippers, or create an alternative device to hold the pen.


- Pen: We used a Sharpie, but you are free to use any pen you like. 
(Note: different pens will need to be configured on the arm to draw correctly.)

- A2 Paper: This is what we used as the surface for the painting. 
(Note: you are free to use an alternative canvas, but it should be configured to allow the pen to draw.)

- [An Ethernet cable](https://www.screwfix.com/c/electrical-lighting/cable/cat8960001?cableproducttype=ethernet) (Optional): We experienced connection issues when connecting to the robot wirelessly, and this improved when using an Ethernet cable.

- [A USB-C to USB converter](https://www.amazon.co.uk/AmazonBasics-Type-C-Gen1-Female-Adapter/dp/B01GGKYYT0/ref=asc_df_B01GGKYYT0/?tag=googshopuk-21&linkCode=df0&hvadid=205231791255&hvpos=&hvnetw=g&hvrand=11493399249999556160&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1006500&hvtargid=pla-490647422152&psc=1&th=1&psc=1) (Optional): Some Mac machines have no USB port. If you are dual booting on a Mac, you may need a converter to boot the Ubuntu image.


### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/github_username/repo_name.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

[label](Assets/linkedin_video.mp4)

_For more examples, please refer to the [Documentation](https://example.com)_


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](https://github.com/github_username/repo_name/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


