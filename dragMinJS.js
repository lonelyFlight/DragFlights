function convertUnit(e,t,a){e=parseFloat(e);var i=1;switch(t){case"km":i=1e3;break;case"cm":i=.01;break;case"in":i=.0254;break;case"ft":i=.3048;break;case"km/h":i=1e3/3600;break;case"ft/s":i=.3048;break;case"mph":i=.44704;break;case"ft/s^2":i=.3048;break;case"grams":i=.001;break;case"lbs":i=.453592;break;case"ounces":i=.02835;break;case"cm^2":i=1e-4;break;case"in^2":i=64516e-8;break;case"ft^2":i=.092903;break;case"degrees":i=Math.PI/180}return e="to"==a?e*=i:e/=i}function dragFlight(e){var t=new Date;e.initVel<0&&(e.initVel*=-1,e.windAngle+=Math.PI);var a=e.mass,n=.5*e.dragCo*e.ortho*e.rho,r=0,l=0-e.gravity,o=Math.cos(e.initAngle)*e.initVel,s=Math.sin(e.initAngle)*e.initVel,h=0,c=e.initHeight,g=e.dt,p=Math.cos(e.windAngle)*e.windVel,u=Math.sin(e.windAngle)*e.windVel,d=0,m=0,v=0,f=0,b=[[h,c]];if(dragAccel=Math.sin(e.windAngle)*n*(Math.pow(p,2)+Math.pow(u,2))/a,dragAccel>e.gravity)return{displacement:"impossible"};for(;c>=0;)h+=o*g,c+=s*g,c>f&&(f=c),o+=r*g,s+=l*g,v=n*(Math.pow(o-p,2)+Math.pow(s-u,2))/a,m=Math.atan2(s-u,o-p),r=-v*Math.cos(m),l=-v*Math.sin(m)-e.gravity,d+=g,b.push([h,c]);var $=new Date,w=$.getTime()-t.getTime(),y=Math.sin(e.initAngle)*e.initVel,M=e.initVel*Math.cos(e.initAngle)*(y+Math.sqrt(Math.pow(y,2)+2*e.gravity*e.initHeight))/e.gravity,k=Math.abs((M-h)/M),I=[b[0]],C=b.length;if(C>100){var D=Math.floor(C/100);for(i=D;i<C;i+=D)I.push(b[i]);I.push(b[C-2])}else I=b;var T={displacement:h,maxHeight:f,flightTime:d,deviation:k,iterations:Math.round(d/g),computationTime:w,pathPlot:I};return T}function idealFlight(e){var t=Math.sin(e.initAngle)*e.initVel,a=(t+Math.sqrt(Math.pow(t,2)+2*e.gravity*e.initHeight))/e.gravity,n=a/100,r=[[0,e.initHeight]],l=Math.cos(e.initAngle)*e.initVel,o=Math.sin(e.initAngle)*e.initVel;for(i=n;i<a;i+=n)r.push([l*i,e.initHeight+o*i-e.gravity*Math.pow(i,2)/2]);return r.push([l*a,e.initHeight+o*i-e.gravity*Math.pow(a,2)/2]),r}function combineFlights(e,t){for(var a=0;a<e.pathPlot.length;a++)e.pathPlot[a].push(null);for(var a=0;a<t.length;a++)t[a].splice(1,0,null);return e.pathPlot.concat(t)}function createNewChart(e,t){function a(){var t=$("#graph-itself").width(),a=Math.abs(t*e.maxHeight/(e.Displacement<1?1:e.displacement));$("#graph-itself").height(50>a?50:a),e.pathPlot.unshift(["X","Drag Flight"]);var i=google.visualization.arrayToDataTable(e.pathPlot),n={hAxis:{title:"Horizontal Displacement (m)",minValue:0},vAxis:{title:"Height (m)",minValue:0},legend:"none"},r=new google.visualization.ScatterChart(document.getElementById("graph-itself"));r.draw(i,n)}CalledOnce?a():(google.charts.load("current",{packages:["corechart"]}),google.charts.setOnLoadCallback(a),CalledOnce=1)}function createCombinedChart(e,t){function a(){for(var t=$("#graph-itself").width(),a=1,i=1,n=0;n<e.length;n++)e[n][0]>a&&(a=e[n][0]),e[n][1]>i&&(i=e[n][1]),e[n][2]>i&&(i=e[n][2]);var r=Math.abs(t*i/a);$("#graph-itself").height(r),e.unshift(["X","Drag Flight","Ideal Flight"]);var l=google.visualization.arrayToDataTable(e),o={hAxis:{title:"Horizontal Displacement (m)",minValue:0},vAxis:{title:"Height (m)",minValue:0},legend:"none"},s=new google.visualization.ScatterChart(document.getElementById("graph-itself"));s.draw(l,o)}CalledOnce?a():(google.charts.load("current",{packages:["corechart"]}),google.charts.setOnLoadCallback(a),CalledOnce=1)}function specialImg(){function e(){$("#special-message").append(a[r]),r++,r>=a.length&&window.clearInterval(l)}window.innerHeight;$("#contents-wrap").html('<div style="position: absolute; left: 50%;"><div id="special-message" style="position: relative; left: -50%;"></div></div>');var t='Hey Annie Wang! <br> Would you go to prom with me? <br> *Err*... more accurately, can you take me to prom? <br> <br> I kinda know that the answer is already yes, <br> so I guess this a giant formality. <br> Never mind that, <br> "it\'s the thought that counts" - <br> which is usually an exuse... <br> I digress; <br> anyway... you wanna go? <br> ... ... ... ... <br> ... ... ... ... <br> ... ... ... ... <br> ????????????!?!??!?!?!?!?!?!!!?!!!??? <br> (Oooh, I bet the anticipation is painful for those who do not know.) <br> <br> -Ethan Freeburg';t=t.split(" ");for(var a=[],i=0;i<t.length;i++)if("<br>"!=t[i]){var n=t[i];a.push.apply(a,n.split("")),a.push(" ")}else a.push(t[i]);$("#special-message").css({"font-family":"Comic Sans MS","font-size":30*window.innerHeight/979+"px","font-color":"black","text-align":"center"});var r=0,l=window.setInterval(e,60)}function dataGather(){return 1!=$("#prop-mass-value").val()&&2==$("#prop-initvel-value").val()&&13==$("#prop-angle-value").val()&&(myManager.flightBar=!0,specialImg()),p={mass:convertUnit($("#prop-mass-value").val(),$("#prop-mass-unit").val(),"to"),dragCo:parseFloat($("#prop-dc-value").val()),ortho:convertUnit($("#prop-ortho-value").val(),$("#prop-ortho-unit").val(),"to"),initVel:convertUnit($("#prop-initvel-value").val(),$("#prop-initvel-unit").val(),"to"),initAngle:convertUnit($("#prop-angle-value").val(),$("#prop-angle-unit").val(),"to"),initHeight:convertUnit($("#prop-initheight-value").val(),$("#prop-initheight-unit").val(),"to"),rho:convertUnit($("#prop-density-value").val(),$("#prop-density-unit").val(),"to"),windVel:convertUnit($("#prop-windvel-value").val(),$("#prop-windvel-unit").val(),"to"),windAngle:convertUnit($("#prop-windangle-value").val(),$("#prop-windangle-unit").val(),"to"),dt:"sec"==$("#prop-iter-unit").val()?$("#prop-iter-value").val():$("#prop-iter-value").val()/1e3,gravity:convertUnit($("#prop-gravity-value").val(),$("#prop-gravity-unit").val(),"to")},p}var UIManager=function(){this.selectedMethod="distance",this.selectedPreset="ideal",this.selectedIdealPathy="omit",this.flightBar=!1,this.changeMethod=function(e){$(".method-options").removeClass("btn-success").addClass("btn-default"),$("#"+e).addClass("btn-success").removeClass("btn-default"),this.selectedMethod=e,"angle"==e?$("#prop-angle-value").attr("disabled",!0):$("#prop-angle-value").attr("disabled",!1)},this.changePresets=function(e){$(".presets").removeClass("btn-success").addClass("btn-default"),"clear"!=e&&$("#"+e).addClass("btn-success").removeClass("btn-default"),this.selectedPreset=e;var t;switch(e){case"clear":t={a:"clr",b:"clr",c:"clr",d:"clr",e:"clr",f:"clr",g:"clr",h:"clr",i:"clr",j:"clr",k:"clr",l:"clr",m:"clr",n:"clr",o:"clr",p:"clr",q:"clr",r:"clr",s:"clr",t:"clr",u:"clr"};break;case"golf":t={a:"43.93",b:"grams",c:"0.3",d:"14.32",e:"cm^2",f:"",g:"",h:"",i:"",j:"",k:"",l:"1.225",m:"kg/m^3",n:"",o:"",p:"",q:"",r:"",s:"",t:"",u:""};break;case"tennis":t={a:"58.1",b:"grams",c:"0.507",d:"35.2565235549",e:"cm^2",f:"",g:"",h:"",i:"",j:"",k:"",l:"1.225",m:"kg/m^3",n:"",o:"",p:"",q:"",r:"",s:"",t:"",u:""};break;case"ideal":t={a:"1.00",b:"kg",c:"0",d:"0",e:"m^2",f:"10",g:"m/s",h:"45",i:"degrees",j:"0",k:"m",l:"0",m:"kg/m^3",n:"0",o:"m/s",p:"0",q:"degrees",r:"1",s:"milisec",t:"9.80665",u:"m/s^2"}}$("#prop-mass-value").val("clr"==t.a?"":t.a),"clr"!=t.b&&$("#prop-mass-unit").val(t.b),$("#prop-dc-value").val("clr"==t.c?"":t.c),$("#prop-ortho-value").val("clr"==t.d?"":t.d),"clr"!=t.b&&$("#prop-ortho-unit").val(t.e),$("#prop-density-value").val("clr"==t.l?"":t.l),"ideal"==e&&$("#prop-density-unit").val(t.m)},this.changeComputeIdeal=function(e){$(".compute-Ideal").removeClass("btn-success").addClass("btn-default"),$("#"+e).addClass("btn-success").removeClass("btn-default"),this.selectedIdealPathy=e},this.changeRunButton=function(){$("#run").addClass("btn-warning").removeClass("btn-default btn-success")},this.postFlightData=function(e,t,a){var i=" m";switch($("#error-text").text(""),$("#prop-initvel-unit").val()){case"km/h":e.displacement>2e3&&(e.displacement=convertUnit(e.displacement,"km","from"),e.maxHeight=convertUnit(e.maxHeight,"km","from"),i=" km");break;case"ft/s":e.displacement=convertUnit(e.displacement,"ft","from"),e.maxHeight=convertUnit(e.maxHeight,"ft","from"),i=" ft";break;case"mph":e.displacement=convertUnit(e.displacement,"ft","from"),e.maxHeight=convertUnit(e.maxHeight,"ft","from"),i=" ft"}if(t){var n=$("#prop-angle-unit").val();$("#prop-angle-value").val(Math.round(100*convertUnit(t,n,"from"))/100)}$("#output-disp").html(Math.round(1e4*e.displacement)/1e4+i),$("#output-maxheight").html(Math.round(1e4*e.maxHeight)/1e4+i),$("#output-flighttime").html(Math.round(1e4*e.flightTime)/1e4+" seconds"),$("#output-deviation").html(Math.round(1e4*e.deviation)/100+"%"),$("#output-iterations").html(Math.round(1e4*e.iterations)/1e4),$("#output-computationTime").html(Math.round(1e4*e.computationTime)/1e4+" milliseconds"),a?createCombinedChart(a,i):createNewChart(e,i),$("#graph").css("display","block"),$("#outputs").css("display","block"),$("#run").addClass("btn-success").removeClass("btn-default btn-warning")},this.runError=function(e){$("#error-text").text(e)},this.validateInput=function(){function e(e){e.message="<p>"+e.message+"</p>",$("#"+e.formGroup).addClass("has-error"),$("#"+e.helpID).css("display","block").html(e.message),myManager.flightBar=!0}function t(){$(".form-group").removeClass("has-error"),$(".validation-error").css("display","none").addClass("text-right").html("")}function a(t){var a=[t.indexOf("-"),t.lastIndexOf("-")],i=t.substring(a[0]+1,a[1]);e({formGroup:"form-"+i,helpID:"help-"+i,message:"The value must be a real number."})}myManager.flightBar=!1,t(),$("input.form-control").each(function(){$.isNumeric(this.value)||a(this.id)}),$("#prop-mass-value").val()<=0&&e({formGroup:"form-mass",helpID:"help-mass",message:"The mass of the object must be greater than 0."}),$("#prop-dc-value").val()<0&&e({formGroup:"form-dc",helpID:"help-dc",message:"The drag coefficient of the object must be greater than or equal to 0."}),$("#prop-ortho-value").val()<0&&e({formGroup:"form-ortho",helpID:"help-ortho",message:"The orthographic projection of the object must be greater than or equal to 0."}),"degrees"==$("#prop-angle-unit").val()?($("#prop-angle-value").val()<0||$("#prop-angle-value").val()>90)&&e({formGroup:"form-angle",helpID:"help-angle",message:"The initial angle must be between 0 and 90 degrees."}):($("#prop-angle-value").val()<0||$("#prop-angle-value").val()>Math.PI/2)&&e({formGroup:"form-angle",helpID:"help-angle",message:"The initial angle must be between 0 and 1/2 PI radians."}),$("#prop-initheight-value").val()<0&&e({formGroup:"form-initheight",helpID:"help-initheight",message:"The initial height of the object must be greater than or equal to 0."}),$("#prop-density-value").val()<0&&e({formGroup:"form-density",helpID:"help-density",message:"The density of the fluid the obejct moves through must be greater than or equal to 0."}),$("#prop-windvel-value").val()<0&&e({formGroup:"form-windvel",helpID:"help-windvel",message:"The velocity of the the wind must be greater than or equal to 0. Add 180 degrees or PI radians to the direction to reverse the direction."}),"degrees"==$("#prop-windangle-unit").val()?($("#prop-windangle-value").val()<0||$("#prop-windangle-value").val()>360)&&e({formGroup:"form-windangle",helpID:"help-windangle",message:"The initial angle must be between 0 and 360 degrees."}):($("#prop-windangle-value").val()<0||$("#prop-windangle-value").val()>2*Math.PI)&&e({formGroup:"form-windangle",helpID:"help-windangle",message:"The initial angle must be between 0 and 2 PI radians."}),$("#prop-gravity-value").val()<=0&&e({formGroup:"form-gravity",helpID:"help-gravity",message:"Gravity must be greater than 0"})}},CalledOnce=0,myManager;$(document).ready(function(){myManager=new UIManager,$(".method-options").click(function(){myManager.changeMethod($(this).attr("id"))}),$(".presets").click(function(){myManager.changePresets($(this).attr("id"))}),$(".compute-Ideal").click(function(){myManager.changeComputeIdeal($(this).attr("id"))}),$("#run").click(function(){myManager.changeRunButton();var e=dataGather();if(myManager.flightBar)return console.log("There was an error in the input!"),void myManager.runError("Looks like there's a problem with one or more inputs. Follow the suggestions to fix the problem, then run the simulation again!");var t=null,a=null;if(a=dragFlight(e),"impossible"==a.displacement)return console.log("Handling Impossiblility"),void myManager.runError("Looks like the flight couldn't be worked out because the force from the wind was greater than gravity! Try decreasing the wind velocity or changing its direction, then hit compute again!");if("angle"==myManager.selectedMethod){var i=0,n=0,r=-1,l=new Date;n+=.1,e.initAngle=convertUnit(n,"degrees","to"),r=i,a=dragFlight(e),i<a.displacement&&(i=a.displacement);for(var o=[],s=[];89.9>n;)n+=.1,e.initAngle=convertUnit(n,"degrees","to"),r=i,a=dragFlight(e),i<a.displacement&&(i=a.displacement),o.push(a.displacement),s.push(n);var h=o.indexOf(i);n=s[h],e.initAngle=convertUnit(n,"degrees","to"),a=dragFlight(e);var c=new Date;a.computationTime=c.getTime()-l.getTime(),t=convertUnit(n,"degrees","to")}else a=dragFlight(e);if("omit"==myManager.selectedIdealPathy)myManager.postFlightData(a,t,null);else{var g=a,p=idealFlight(e);myManager.postFlightData(g,t,combineFlights(g,p))}}),$(".form-control").blur(function(){myManager.validateInput()})});
