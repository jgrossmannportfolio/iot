var myapp = (function(){
    var displacementData = []; // preallocation for data reception from parse
    var gyroData = [];
    var canvasID = '';          // to be loaded with the canvas ID given by the corresponding page

    var puttGain = 0;
    var xPuttGain = 0;      // z direction for graphics (i.e. in and out)
    var yPuttGain = 0;   // x direction for graphics (i.e. left to right))
    var zPuttGain = 0;   // y direction for graphics (i.e. up and down)
    var putterHeight = 0.83;  // in meters
    var delta_t = 12.5;                   // frame rate, in msec
    var worldScalingFactor = 0;         // canvas world scaling factor, will be set in setCanvasDim() 
    var pixelsPerMeter = 3779.527559055;// conversion
    var canvasPixelsPerMeter = 0;       // will be set in setCanvasDim()


    function setCanvasDim(canvasID) {
        width = window.innerWidth;
        height = window.innerHeight;
        $("#"+canvasID).attr("width", width);
        $("#"+canvasID).attr("height", height / 2.0);
        console.log("width: "+$("#"+canvasID).attr("width"));
        console.log("height: "+$("#"+canvasID).attr("height"));
        //worldScalingFactor = (height/2)/(10*pixelsPerMeter);
        //worldScalingFactor = (Math.pow(height,2)/2)/(Math.pow(pixelsPerMeter,2));
        worldScalingFactor = 1/100;
        console.log(worldScalingFactor);
        canvasPixelsPerMeter = worldScalingFactor*pixelsPerMeter;
        console.log("Canvas Pixels per Meter = ");
        console.log(canvasPixelsPerMeter);
        console.log("set canvas dimensions");
    };

    function getPuttData() {
        if($("#puttCanvas").length) {
            canvasID = 'puttCanvas';
            console.log("found canvas");
            setCanvasDim(canvasID);
            
            console.log("getting put data");
            var putt = Parse.Object.extend("Putt");
            var query = new Parse.Query(putt);
            query.get("12fz4AHTDK", {
                success: function(data) {
                    
                        // receive data
                        displacementData = data.get("frames");
                        gyroData = data.get("gyro");
                    
                        displacementData = parseIncomingArray(displacementData);
                        displacementData = changeSigns(displacementData); //change incoming y
                        gyroData = parseIncomingArray(gyroData);
                        
                        // data processing routine for translation graphics
                        displacementData = applyZoffset(displacementData); 

                        // converting data to pixels
                        xPuttGain = puttGain;      // z direction for graphics (i.e. in and out)
                        yPuttGain = puttGain;   // x direction for graphics (i.e. left to right))
                        zPuttGain = puttGain;   // y direction for graphics (i.e. up and down)
                        displacementData = toPixels(displacementData,canvasPixelsPerMeter);  
                        
                        console.log("processed data (gyro, displacement)");
                        console.log(gyroData);
                        console.log(displacementData);
                    
                    // jump to animation once the data has been received and processed
                    putterDemo(canvasID,displacementData,gyroData);   


                },
                error: function(object, error) {
                    console.log("Error getting putt data");
                    console.log(error);

                }

            });
        
        }
        // if SAMPLE PUTT ANIMATION page was rendered
        if($("#samplePuttCanvas").length) {
            
            canvasID = 'samplePuttCanvas';
            console.log("found canvas: "+canvasID);
            setCanvasDim(canvasID);
            

            /*[gyroData, displacementData] = generateRandPath();
            [gyroData, displacementData] = generatePutt();
            displacementData = changeSigns(displacementData, 1); //change incoming y
            displacementData = applyZoffset(displacementData);  
            displacementData = toPixels(displacementData,canvasPixelsPerMeter); 
            putterDemo(canvasID,displacementData,gyroData); 
*/
            
            // Grab sample putt data from parse object
            var putt = Parse.Object.extend("Putt");
            var query = new Parse.Query(putt);
            query.get("12fz4AHTDK", {
                success: function(data) {
                    // receive data
                    displacementData = data.get("samplePuttDisplacement");
                    displacementData = changeSigns(displacementData, 1); //change incoming y
                    gyroData = data.get("sampleGyroDisplacement");
                    displacementData = applyZoffset(displacementData);
                    xPuttGain = 4;      // z direction for graphics (i.e. in and out)
                    yPuttGain = 4;   // x direction for graphics (i.e. left to right))
                    zPuttGain = 4;   // y direction for graphics (i.e. up and down)
                    displacementData = toPixels(displacementData,canvasPixelsPerMeter); 
                    
                    console.log(displacementData);
                    console.log(gyroData);

                    putterDemo(canvasID,displacementData,gyroData); 
                }, error:  function(error) {
                    alert("error with .get");}
                }); 


            // posting sample data to the parseCloud "core" dashboard (in m/s)
            /*
            [gyroData, displacementData] = generatePutt();
        
            var puttClass = Parse.Object.extend("Putt");
            var putt2 = new puttClass();
            putt2.id = "12fz4AHTDK";

            putt2.set("samplePuttDisplacement", displacementData);
            putt2.set("sampleGyroDisplacement", gyroData);
            putt2.save(null, {
              success: function(putt2) {
                console.log("success!!!");
                displacementData = changeSigns(displacementData, 1); //change incoming y
                displacementData = applyZoffset(displacementData);

                xPuttGain = 4;      // z direction for graphics (i.e. in and out)
                yPuttGain = 4;   // x direction for graphics (i.e. left to right))
                zPuttGain = 4;   // y direction for graphics (i.e. up and down)
                displacementData = toPixels(displacementData,canvasPixelsPerMeter); 
                putterDemo(canvasID,displacementData,gyroData); 

              }
            });*/
            

        }


        function generatePutt(){
            var randIter = 1,
                displacement = [[0,0,0]],
                rotations = [[0,0,0]],
                scale = 1/2,    // back swing accel to ball contact accel ratio
                xx = (0.6/10)/canvasPixelsPerMeter, // y for graphics  (pos)
                yy = -(0.6/10)/canvasPixelsPerMeter, // z for graphics (neg)
                zz = (10/10)/canvasPixelsPerMeter,   // x for graphics (pos)
                xr = -0.6,
                yr = -0.3,
                        duration = [ 0, 50,(scale)*50,(scale)*50,(scale/2)*50],
            yPositionincrementer = [ -xx, (1/scale)*xx, -(1/(scale))*xx, -(1/(scale))*xx],
            zPositionincrementer = [ (1/(scale))*yy, -(1/(scale))*yy, (1/(scale))*(yy), (1/(scale))*(yy)],
            xPositionincrementer = [zz, -(1/scale)*zz, -(1/scale)*zz,-(1/scale)*zz  ],
                
            yRotationincrementer = [ -xr, (1/scale)*xr,  (1/(scale))*xr, xr], //hook
            zRotationincrementer = [ -yr, (1/scale)*yr, (1/(scale))*yr, yr], //bent wrists before putt
            xRotationincrementer = [ 0, 0,  0, 0],
                                        
            counter = 1,
            ii = 1;
            for (ii; ii < duration.length; ii=ii+1){
                console.log("ii");
                console.log(ii);
                ind = ii-1;
                for (randIter=counter; randIter < duration[ii]+counter; randIter = randIter+1 ){
                    prevInd = randIter-1;
                    xprev = displacement[prevInd][0];
                    yprev = displacement[prevInd][1];
                    zprev = displacement[prevInd][2];
                    displacement.push([xprev-xPositionincrementer[ind],
                                       yprev-yPositionincrementer[ind],
                                       zprev-zPositionincrementer[ind]]);

                    rotations.push([rotations[randIter-1][0] + xRotationincrementer[ind],
                                    rotations[randIter-1][1] + yRotationincrementer[ind],
                                    rotations[randIter-1][2] + zRotationincrementer[ind]]);
                    //console.log("innerloop");
                }
                counter = counter + duration[ii]; // update total iteration count
                console.log("counter");
                console.log(counter);
            }
            
            return [rotations, displacement];
        }

        
        function generateRandPath(){
            var randIter = 1,
                displacement = [[0,0,0]],
                gyroData = [[0,0,0]],
                xPositionincrementer = 0.0,
                yPositionincrementer = -0.0,
                zPositionincrementer = 0.1;
              
            for (randIter; randIter < 100; randIter = randIter + 1 ){
                xprev = displacement[randIter-1][0];
                yprev = displacement[randIter-1][1];
                zprev = displacement[randIter-1][2];
                displacement.push([xprev-xPositionincrementer,yprev-yPositionincrementer,zprev-zPositionincrementer]);
                gyroData.push([0,0,0]);
            }
            for (randIter; randIter < 200; randIter = randIter + 1 ){
                xprev = displacement[randIter-1][0];
                yprev = displacement[randIter-1][1];
                zprev = displacement[randIter-1][2];
                displacement.push([xprev+xPositionincrementer,yprev+yPositionincrementer,zprev+zPositionincrementer]);
                gyroData.push([0,0,0]);
            }
            return [gyroData, displacement];
        }

        
    }; // ends getPuttData()


       // ======== functions for data  ============
        function changeSigns (incomingArray, ind){
            jj = 0;
            for (jj; jj < incomingArray.length; jj++){
                incomingArray[jj][ind] = (-1)*incomingArray[jj][ind];
            }
            return incomingArray;
        }

        
        function parseIncomingArray (incomingArray){
            if (incomingArray[0].length != 3){
                // append the incorrect parsings as the first array in the array
                console.log("python bug accounted for");   
                firstArray = [[incomingArray[0],incomingArray[1],incomingArray[2]]];
                incomingArray = incomingArray.slice(3, incomingArray.length);
                incomingArray = firstArray.concat(incomingArray);
            }
            return incomingArray;
        }


        function applyZoffset(displacementframes){ 
        // applies the z offset to the displacement frames from the sensor
        //   -> returns the new displacement frames
          //var displacementframes = displacementframes;
          console.log("applyZoffset()");
          newframes = [[0,0,0]];

            //Create Z-offset matrix via screen size
            pageWidth = $("#"+canvasID).attr("width")-0; //subtraction converts str to int
            pageHeight = $("#"+canvasID).attr("height")-0;
            if (pageWidth > pageHeight ){
                z1 = .5;
                z2 = pageHeight/(2*pageWidth);
            } else {
                z1 = pageWidth/(pageWidth+pageHeight);
                z2 = pageHeight/(pageWidth+pageHeight);
            }
            z3 = -1;
            Zoffset = [z1,z2,z3]; 

        // iterate through old frames
          var j = 1;  
          for (j = 1; j< displacementframes.length; j++){


            // Calculate position increment from displacement data
            xdisp = displacementframes[j][0] - displacementframes[j-1][0];
            ydisp = displacementframes[j][1] - displacementframes[j-1][1];
            zdisp = displacementframes[j][2] - displacementframes[j-1][2];
            
            // take into account z bias and position increments
            yincrementer = ydisp + Zoffset[0]*(xdisp);
            zincrementer = zdisp + Zoffset[1]*(xdisp);
            xincrementer = Zoffset[2]*(xdisp);
        
            // update displacement frames based on z bias
            newframesY = newframes[j-1][1] + yincrementer;
            newframesZ = newframes[j-1][2] + zincrementer;
            newframesX = newframes[j-1][0] + xincrementer;
            
            // append new array
            newframes.push([newframesX,newframesY,newframesZ]);    
          }

          return newframes;
       } // ends applyZoffset()


      function toPixels(displacementframes,scale){
            console.log("toPixels()");
            var ii = 0,
                newframes = [];        
            for (ii; ii < displacementframes.length; ii++){
                //function forloop (ii){
                    pixX = xPuttGain*scale*displacementframes[ii][0];
                    pixY = yPuttGain*scale*displacementframes[ii][1];
                    pixZ = zPuttGain*scale*displacementframes[ii][2];
                    newframes.push([pixX,pixY,pixZ]);
                //};
            }

            return newframes;
       }
            


    function BTconnect() { // connect with the listening wiced sense sensor


          var connect = "true";

          Parse.Push.send({
            channels: ["Putt"],
            data: {"Putt":connect}
          }, 
          {success: function() {
                //alert("Push Successful! BTconnect = true");
                 /*   
                    var putt = Parse.Object.extend("Putt");
                    var query = new Parse.Query(putt);
                    query.get("12fz4AHTDK", {
                        success: function(data) {
                            // receive data
                            connect = data.get("connect2wiced");
                            alert("Push Successful -> connect2wiced value received = " + connect);
                        }, error:  function(error) {
                            alert("error with .get");}
                        }); */
            }, error: function(error) {
                 alert("Error" + error);
            }
          });




    };



// ----------------------------------------------------


  function buildPutter(g, height, colors){
    console.log("buildingputter()");
    // height input in pixels scaled to the canvas height
    // Customize your putter
    buttDisplacementScalar = 6; // essentially the x displacement from tip of the shaft to the putter head
    putterHeightScalar = 17;
    width = height/putterHeightScalar;
    
    var sq = [['M',0,0,0, 'L',width*4,0,0, width*4,width,0, 0,width,0, 'z']];
      sq.push(sq[0]);
      sq.push(sq[0]);
      sq.push(sq[0]);
      sq.push(['M',0,0,0, 'L',width,0,0, width,width,0, 0,width,0, 'z']);
      sq.push(sq[4]);
      sq.push(['M',0,0,0, 'L',width/2,0,0, width/2,width/2,0, 0,width/2,0, 'z']); // dot on putter head
    
      var faces = g.createGroup3D(),
      side,
      translateX = [-width*2,-width*2,-width*2,-width*2,-width/2,-width/2,-width/4],
      translateY = [0,-width/2,-width/2, -width,0,0,width/4],
      translateZ = [width/2,0,width,width/2,width*2,width*2,width/2+1],
      rotateX = [1,1,1,1,0,0,0],
      rotateY = [1,0,0,0,1,1,1],
      rotateZ = [1,0,0,0,0,0,0],
      rotateMag = [0,90,-90,180,90,-90,180],
      i;

    for (i=0; i<7; i++){
        //console.log(i)
        side = g.compileShape3D(sq[i], colors[i]);
        side.translate(translateX[i], translateY[i], translateZ[i]);
        side.rotate(rotateX[i], rotateY[i], rotateZ[i], rotateMag[i]);
        //side.backHidden = true;
        faces.addObj(side);
    }
    
    shaft2ShaftPlusGripRatio = 7/10;
    // creating the shaft
    lowerShaft = g.compilePath3D(["M",-width,width,0, "L",-width,2*width,0], "#A8A8A8", width/3);
    faces.addObj(lowerShaft);
      
    upperShaft = g.compilePath3D(["M",-width,2*width,0, "L",(shaft2ShaftPlusGripRatio)*(-buttDisplacementScalar*width),(shaft2ShaftPlusGripRatio)*(putterHeightScalar*width),0], "#A8A8A8", width/4);
    faces.addObj(upperShaft);
      
    grip = g.compilePath3D(["M",(shaft2ShaftPlusGripRatio)*(-buttDisplacementScalar*width),(shaft2ShaftPlusGripRatio)*(putterHeightScalar*width),0, "L",(-buttDisplacementScalar*width),(putterHeightScalar*width),0], "#202020", width/3);
    faces.addObj(grip);
      
    // reference putter head to the center
    faces.translate(0,-width/2,0);
      
    // add axes for visual debugging
    centerX = g.compilePath3D(["M",-4*width,0,0, "L",4*width,0,0], "green", 1);
    faces.addObj(centerX);
    centerY = g.compilePath3D(["M",0,-4*width,0, "L",0,4*width,0], "green", 1);
    faces.addObj(centerY);
    centerZ = g.compilePath3D(["M",0,0,-4*width, "L",0,0,4*width], "green", 1);
    faces.addObj(centerZ);

    
    return faces;
  }





// ============================ MOVE PUTTER ==================================
  function putterDemo(scrnID,displacementData,gyroData)
  {
    var g = new Cango3D(scrnID),
      newPos = {x:0, y:0, z:0},  // avoid creating Objects in event handlers
      taggedFace, cube1,
      width = 20,
      colors = ["#686868","#686868","#686868","#686868","#686868","#686868", "red"],
      iter = 0,
      timer = 0,
      coordsX = -($("#"+canvasID).attr("width")/2), // bottom leftmost pixel x location
      coordsY = -($("#"+canvasID).attr("height")/2), // bottom leftmose pixel y location
      xyspan = -(2*coordsX);
      console.log(g.cnvs.offsetWidth);

    // Initialize subfunction within putterDemo()
      function movePutter() {
            //console.log(iter);
        
            function nextframe (convertedframes){
            // note that z graphics are in towards the screen, thus Z and Y are switched
                X = convertedframes[iter][1];
                Y = convertedframes[iter][2];
                Z = convertedframes[iter][0];

                //g.setWorldCoords3D(coordsX+X, coordsY+Y, xyspan+Z);
                g.setWorldCoords3D(coordsX-X, coordsY-Y, xyspan-Z);
            }

            var useDataMode = 1; // set this to one to use parse data
            
            if (useDataMode){   // USEdata mode
                if (iter == 0){
                    cube1.transform.rotate(0,0,1,gyroData[iter][0]);
                    cube1.transform.rotate(1,0,0,gyroData[iter][1]);
                    cube1.transform.rotate(0,1,0,gyroData[iter][2]);
                    nextframe(displacementData); // sets world coordinates based on frame

                } else if (iter == gyroData.length - 1) {
                    iter = -1; // reset the array counter for loop
                    //cube1.transform.reset(); // reset the 3d object

                    clearInterval(timer);
                        // refresh data if new putt
                    console.log(canvasID);
                        //if ( !(canvasID.localeCompare('puttCanvas')) ){
                        if (canvasID == 'puttCanvas'){
                            console.log("pull form parse");
                            console.log("getting put data");
                            var putt = Parse.Object.extend("Putt");
                            var query = new Parse.Query(putt);
                            query.get("12fz4AHTDK", {
                                success: function(data) {
                        
                                    // receive data
                                    displacementData = data.get("frames");
                                    gyroData = data.get("gyro");
                                
                                    //displacementData = parseIncomingArray(displacementData);
                                    displacementData = changeSigns(displacementData); //change incoming y
                                    gyroData = parseIncomingArray(gyroData);
                                    
                                    // data processing routine for translation graphics
                                    displacementData = applyZoffset(displacementData); 

                                    // converting data to pixels
                                    xPuttGain = puttGain;      // z direction for graphics (i.e. in and out)
                                    yPuttGain = puttGain;   // x direction for graphics (i.e. left to right))
                                    zPuttGain = puttGain;   // y direction for graphics (i.e. up and down)
                                    displacementData = toPixels(displacementData,canvasPixelsPerMeter);  
                                    
                                    console.log("processed data (gyro, displacement)");
                                    console.log(gyroData);
                                    console.log(displacementData);
                            
                                },
                                error: function(object, error) {
                                    console.log("Error getting putt data");
                                    console.log(error);

                                }

                            });
                        } //else if(canvasID == )*/


                    setTimeout(function() {


                        cube1.transform.reset(); // reset the 3d object
                        timer = setInterval(movePutter, delta_t);
                    }, 500);







                    
                } else {
                // subtract here because gyroData comes in as angular displacement, Cango accumulates
                    cube1.transform.rotate(0,0,1,-(gyroData[iter][0] - gyroData[iter-1][0]));
                    cube1.transform.rotate(1,0,0,-(gyroData[iter][1] - gyroData[iter-1][1]));
                    cube1.transform.rotate(0,1,0,-(gyroData[iter][2] - gyroData[iter-1][2]));
                    nextframe(displacementData); // sets world coordinates based on frame
                }

                iter = iter + 1; // increment the array counter

            } else{ // not in USEdata mode
             cube1.transform.rotate(1,0,0,10);
            }

            g.renderFrame(cube1); // update the canvas
      } // ends movePutter()


      g.setWorldCoords3D(coordsX, coordsY, xyspan);
      g.setFOV(45);
      g.setPropertyDefault("backgroundColor", "lightyellow");
    
      // build putter object
      //putterHeight = .83;  // in meters
      cube1 = buildPutter(g, canvasPixelsPerMeter*(putterHeight*10), colors); // note for builfing cube/putter: len(colors1) != len(colors0)
      console.log("Putter Built");
      timer = setInterval(movePutter, delta_t); // in msec
  } 



    // return from the outermost function, myapp
    return {
        init: function() {
            console.log("starting up");
            Parse.initialize("iAFEw9XwderX692l0DGIwoDDHcLTGMGtcBFbgMqb", "d4q6T1YxleQ2za17PdTtfaAD8x6tAB9rdW4y9vMD");


            window.requestAnimFrame = (function(callback) {
                return window.requestAnimationFrame || window.webkitRequestAnimationFrame || window.mozRequestAnimationFrame || window.oRequestAnimationFrame || window.msRequestAnimationFrame ||
                function(callback) {
                  window.setTimeout(callback, 1000 / 60);
                };
            })();



            $(window).load(getPuttData);
            jQuery("#startButton").click(BTconnect);
            //$(window).load(putterDemo("puttCanvas"))
            //console.log("Client-side app starting up")
            
			//jQuery("#testbutton").click(test);
            
        }
    }
})();
jQuery(myapp.init); // render the outermost function


