//Function convertToSI - converts value 'to' or 'from' SI units; direction indicates conversion
function convertUnit(value,unit,direction){
  value = parseFloat(value);
  var factor = 1;
  switch(unit){
    //length - m
    case 'km': factor = 1000; break;
    case 'cm': factor = 0.01; break;
    case 'in': factor = 0.0254; break;
    case 'ft': factor = 0.3048; break;

    //velocity - m/s
    case 'km/h': factor = 1000 / 3600; break;
    case 'ft/s': factor = 0.3048; break;
    case 'mph': factor = 0.44704; break;

    //acceleration - m/s^2
    case 'ft/s^2': factor = 0.3048; break;

    //mass - kg
    case 'grams': factor = 0.001; break;
    case 'lbs': factor = 0.453592; break;
    case 'ounces': factor = 0.02835; break;

    //area - m^2
    case 'cm^2': factor = 0.0001; break;
    case 'in^2': factor = 0.00064516; break;
    case 'ft^2': factor = 0.092903; break;

    //angle - radians
    case 'degrees': factor = Math.PI / 180; break;

  }
  value = (direction=='to') ? value *= factor : value /= factor;
  return value;
}

//Function DragFlight - computes and returns drag flight properites and path
function dragFlight(p){
  //  console.log(p);
    var tick = new Date();

    //set up for iterating

    if(p.initVel < 0){
      p.initVel *= -1;
      p.windAngle += Math.PI;
    }

    var mass = p.mass;
    var b = 0.5 * p.dragCo * p.ortho * p.rho;
    var ax = 0;
    var ay = 0 - p.gravity;
    var vx = Math.cos(p.initAngle) * p.initVel;
    var vy = Math.sin(p.initAngle) * p.initVel;
    var sx = 0;
    var sy = p.initHeight;
    var dt = p.dt;
    var wvx = Math.cos(p.windAngle) * p.windVel;
    var wvy = Math.sin(p.windAngle) * p.windVel;
    var t = 0;
    var vm = 0;
    var force = 0;
    var angle = 0;
    var dragaccel = 0;
    var maxsy = 0;

    var pathplot = [ [sx , sy ] ];

    //Check for impossible wind
    dragAccel = Math.sin(p.windAngle) * b * ( Math.pow( (wvx) ,2) + Math.pow( (wvy) ,2) ) / mass;
    if( dragAccel > p.gravity){
      return { displacement: 'impossible' };
    }
    //console.log('Completeed instanciation');

    while( sy>=0 ){
      sx += vx * dt;
      sy += vy * dt;

      if( sy > maxsy ){
        maxsy = sy;
      }

      vx += ax * dt;
      vy += ay * dt;

      dragaccel = b * ( Math.pow( (vx - wvx) ,2) + Math.pow( (vy-wvy) ,2) ) / mass;

      angle = Math.atan2( (vy-wvy), (vx-wvx) );
      ax = -dragaccel*Math.cos(angle);
      ay = -dragaccel*Math.sin(angle) - p.gravity;

      t += dt;

      pathplot.push([ sx , sy]);
      //console.log('SX: ' + sx + " SY: " + sy);
    }

    //Determines computation time
    var tock = new Date();
    var elapsedTime = tock.getTime() - tick.getTime();

    //determines error
    var B = Math.sin(p.initAngle) * p.initVel;
    var ideality = p.initVel * Math.cos(p.initAngle) * ( B + Math.sqrt( Math.pow(B,2) + 2 * p.gravity * p.initHeight ) ) / p.gravity;
    //console.log('DEBUG IDEAL: ' + ideality);
    var error = Math.abs( (ideality - sx) / ideality);

    //repacks pathplot to limit it to 101 spots
    var smallPlot = [ pathplot[0] ];
    var num = pathplot.length;
    if( num > 100){
      var dindex = Math.floor( num / 100 );
      for(i = dindex; i < num; i += dindex){
        smallPlot.push( pathplot[i] );
      }
      smallPlot.push( pathplot[ num - 2 ]);
    } else {
      smallPlot = pathplot;
    }


    //packs return
    var info = {
      displacement: sx,
      maxHeight: maxsy,
      flightTime: t,
      deviation: error,
      iterations: Math.round( t/dt ),
      computationTime: elapsedTime,
      pathPlot: smallPlot
    }
    return info;
}

//Function IdealFlight - computes and returns ideal flight path
function idealFlight(p){
  //var t = 2 * Math.sin(p.initAngle) * p.initVel / p.gravity;
  var B = Math.sin(p.initAngle) * p.initVel;
  var t = ( B + Math.sqrt( Math.pow(B,2) + 2 * p.gravity * p.initHeight ) ) / p.gravity;
  var dt = t / 100;
  var pathPlot = [ [0 , p.initHeight] ];

  var vx = Math.cos(p.initAngle) * p.initVel;
  var vy = Math.sin(p.initAngle) * p.initVel;

  for( i = dt; i < t ; i += dt ){
    pathPlot.push([ (vx * i) , (p.initHeight + vy * i - p.gravity * Math.pow(i,2) / 2) ])
  }
  pathPlot.push([ (vx * t) , (p.initHeight + vy * i - p.gravity * Math.pow(t,2) / 2 )])
  //console.log( pathPlot );
  return pathPlot;
}

//Takes drag flight properties (including path) and the ideal flight and combines them into one flight
function combineFlights( drags , ideals ){
  var hold;
  for(var i = 0; i < drags.pathPlot.length; i++){
    drags.pathPlot[i].push(null);
  }
  for(var i = 0; i < ideals.length; i++){
    ideals[i].splice(1,0,null);
  }
  return drags.pathPlot.concat(ideals);
}

//Object UIManager - Modifies the UI to reflect changes in data
var UIManager = function(){
  this.selectedMethod = 'distance'; //alt. 'angle';
  this.selectedPreset = 'ideal';
  this.selectedIdealPathy = 'omit'; //alt. compute
  this.flightBar = false;


  this.changeMethod = function(target){
    $('.method-options').removeClass('btn-success').addClass('btn-default');
    $('#' + target).addClass('btn-success').removeClass('btn-default');
    this.selectedMethod = target;
    if( target == 'angle'){
      $('#prop-angle-value').attr('disabled',true);
    } else {
      $('#prop-angle-value').attr('disabled',false);
    }
  }

  this.changePresets = function(target){
    $('.presets').removeClass('btn-success').addClass('btn-default');
    if( target != 'clear' ){
      $('#' + target).addClass('btn-success').removeClass('btn-default');
    }
    this.selectedPreset = target;

    var p;
    switch( target ){
      case 'clear':
        p = {
          a: 'clr',
          b: 'clr',
          c: 'clr',
          d: 'clr',
          e: 'clr',
          f: 'clr',
          g: 'clr',
          h: 'clr',
          i: 'clr',
          j: 'clr',
          k: 'clr',
          l: 'clr',
          m: 'clr',
          n: 'clr',
          o: 'clr',
          p: 'clr',
          q: 'clr',
          r: 'clr',
          s: 'clr',
          t: 'clr',
          u: 'clr'
        }
      break;

      case 'golf':
        p = {
          a: '43.93',
          b: 'grams',
          c: '0.3',
          d: '14.32',
          e: 'cm^2',
          f: '',
          g: '',
          h: '',
          i: '',
          j: '',
          k: '',
          l: '1.225',
          m: 'kg/m^3',
          n: '',
          o: '',
          p: '',
          q: '',
          r: '',
          s: '',
          t: '',
          u: ''
        }
      break;

      case 'tennis':
        p = {
          a: '58.1',
          b: 'grams',
          c: '0.507',
          d: '35.2565235549',
          e: 'cm^2',
          f: '',
          g: '',
          h: '',
          i: '',
          j: '',
          k: '',
          l: '1.225',
          m: 'kg/m^3',
          n: '',
          o: '',
          p: '',
          q: '',
          r: '',
          s: '',
          t: '',
          u: ''
        }
      break;

      case 'ideal':
        p = {
          a: '1.00',
          b: 'kg',
          c: '0',
          d: '0',
          e: 'm^2',
          f: '10',
          g: 'm/s',
          h: '45',
          i: 'degrees',
          j: '0',
          k: 'm',
          l: '0',
          m: 'kg/m^3',
          n: '0',
          o: 'm/s',
          p: '0',
          q: 'degrees',
          r: '1',
          s: 'milisec',
          t: '9.80665',
          u: 'm/s^2'
        }
      break;
    }

    //$('#prop-mass-value').val(p.a);
    $('#prop-mass-value').val( (p.a=='clr')?'':p.a );
    if(p.b!='clr')$('#prop-mass-unit').val(p.b);
    $('#prop-dc-value').val((p.c=='clr')?'':p.c);
    $('#prop-ortho-value').val((p.d=='clr')?'':p.d);
    if(p.b!='clr')$('#prop-ortho-unit').val(p.e);
    /*$('#prop-initvel-value').val((p.f=='clr')?'':p.f);
    if(target=='ideal')$('#prop-initvel-unit').val(p.g);
    $('#prop-angle-value').val((p.h=='clr')?'':p.h);
    if(target=='ideal')$('#prop-angle-unit').val(p.i);
    $('#prop-initheight-value').val((p.j=='clr')?'':p.j);
    if(target=='ideal')$('#prop-initheight-unit').val(p.k);*/
    $('#prop-density-value').val((p.l=='clr')?'':p.l);
    if(target=='ideal')$('#prop-density-unit').val(p.m);
    /*$('#prop-windvel-value').val((p.n=='clr')?'':p.n);
    if(target=='ideal')$('#prop-windvel-unit').val(p.o);
    $('#prop-windangle-value').val((p.p=='clr')?'':p.p);
    if(target=='ideal')$('#prop-windangle-unit').val(p.q);
    if(target=='ideal')$('#prop-iter-value').val(p.r);
    if(target=='ideal')$('#prop-iter-unit').val(p.s);
    if(target=='ideal')$('#prop-gravity-value').val(p.t);
    if(target=='ideal')$('#prop-gravity-unit').val(p.u);*/
  }

  this.changeComputeIdeal = function(target){
    $('.compute-Ideal').removeClass('btn-success').addClass('btn-default');
    $('#' + target).addClass('btn-success').removeClass('btn-default');
    this.selectedIdealPathy = target;
  }

  this.changeRunButton = function(){
    $('#run').addClass('btn-warning').removeClass('btn-default btn-success');
  }

  this.postFlightData = function(data , optimalAngle, combinedData){
    var unit = ' m';

    $('#error-text').text('');

    //console.log('DEBUG displacement: '+ data.displacement);
    switch( $('#prop-initvel-unit').val() ){
      case 'km/h':
        if( data.displacement > 2000){
          data.displacement = convertUnit( data.displacement , 'km', 'from' );
          data.maxHeight = convertUnit( data.maxHeight , 'km', 'from' );
          unit = ' km';
        }
      break;

      case 'ft/s':
        data.displacement = convertUnit( data.displacement , 'ft', 'from' );
        data.maxHeight = convertUnit( data.maxHeight , 'ft', 'from' );
        unit = ' ft';
      break;

      case 'mph':
        data.displacement = convertUnit( data.displacement , 'ft', 'from' );
        data.maxHeight = convertUnit( data.maxHeight , 'ft', 'from' );
        unit = ' ft';
      break;

    }

    if(optimalAngle){
      var angleUnit = $('#prop-angle-unit').val();
      $('#prop-angle-value').val( Math.round( convertUnit( optimalAngle , angleUnit , 'from') *100)/100 );
    }

    $('#output-disp').html( (Math.round(data.displacement * 10000) / 10000) + unit);
    $('#output-maxheight').html( (Math.round(data.maxHeight * 10000) / 10000) + unit);
    $('#output-flighttime').html( (Math.round(data.flightTime * 10000) / 10000) + ' seconds');
    $('#output-deviation').html( (Math.round(data.deviation * 10000) / 100) + '%');
    $('#output-iterations').html( (Math.round(data.iterations * 10000) / 10000) );
    $('#output-computationTime').html( (Math.round(data.computationTime * 10000) / 10000) + ' milliseconds');

    if(combinedData){
      createCombinedChart( combinedData, unit);
    } else {
        createNewChart(data, unit);
    }

    $('#graph').css('display','block');
    $('#outputs').css('display','block');
    $('#run').addClass('btn-success').removeClass('btn-default btn-warning');
  }

  this.runError = function(errorText){
    $('#error-text').text(errorText)
  }

  this.validateInput = function(){
    myManager.flightBar = false;
    function addWarning( a ){
      a.message = '<p>' + a.message + '</p>';
      $('#'+a.formGroup).addClass('has-error');
      $('#'+a.helpID).css('display','block').html(a.message);
      myManager.flightBar = true;
    }
    function clearWarnings(){
      $('.form-group').removeClass('has-error')
      $('.validation-error').css('display','none').addClass('text-right').html('');
    }

    function addNonNumericWarning(id){
      var hyphens = [ id.indexOf("-") , id.lastIndexOf("-") ];
      var group = id.substring( hyphens[0]+1 , hyphens[1] );
      addWarning( {
        formGroup: 'form-' + group,
        helpID: 'help-' + group,
        message: 'The value must be a real number.'
      });
    }

    clearWarnings();

    //Determines if all the inputs are numbers;
    $('input.form-control').each(function(){
      if( !$.isNumeric( this.value ) ){
        addNonNumericWarning( this.id );
      }
    });


    if( $('#prop-mass-value').val() <= 0){
      addWarning( {
        formGroup: 'form-mass',
        helpID: 'help-mass',
        message: 'The mass of the object must be greater than 0.'
      } );
    }
          if( $('#prop-dc-value').val() < 0){
            addWarning( {
              formGroup: 'form-dc',
              helpID: 'help-dc',
              message: 'The drag coefficient of the object must be greater than or equal to 0.'
            } );
          }

          if( $('#prop-ortho-value').val() < 0){
            addWarning( {
              formGroup: 'form-ortho',
              helpID: 'help-ortho',
              message: 'The orthographic projection of the object must be greater than or equal to 0.'
            } );
          }

          if( $('#prop-angle-unit').val() == 'degrees'){
            if( $('#prop-angle-value').val() < 0 || $('#prop-angle-value').val() > 90){
              addWarning( {
                formGroup: 'form-angle',
                helpID: 'help-angle',
                message: 'The initial angle must be between 0 and 90 degrees.'
              } );
            }
          } else {
            if( $('#prop-angle-value').val() < 0 || $('#prop-angle-value').val() > Math.PI / 2){
              addWarning( {
                formGroup: 'form-angle',
                helpID: 'help-angle',
                message: 'The initial angle must be between 0 and 1/2 PI radians.'
              } );
            }
          }

          if( $('#prop-initheight-value').val() < 0){
            addWarning( {
              formGroup: 'form-initheight',
              helpID: 'help-initheight',
              message: 'The initial height of the object must be greater than or equal to 0.'
            } );
          }

          if( $('#prop-density-value').val() < 0){
            addWarning( {
              formGroup: 'form-density',
              helpID: 'help-density',
              message: 'The density of the fluid the obejct moves through must be greater than or equal to 0.'
            } );
          }

          if( $('#prop-windvel-value').val() < 0){
            addWarning( {
              formGroup: 'form-windvel',
              helpID: 'help-windvel',
              message: 'The velocity of the the wind must be greater than or equal to 0. Add 180 degrees or PI radians to the direction to reverse the direction.'
            } );
          }

          if( $('#prop-windangle-unit').val() == 'degrees'){
            if( $('#prop-windangle-value').val() < 0 || $('#prop-windangle-value').val() > 360){
              addWarning( {
                formGroup: 'form-windangle',
                helpID: 'help-windangle',
                message: 'The initial angle must be between 0 and 360 degrees.'
              } );
            }
          } else {
            if( $('#prop-windangle-value').val() < 0 || $('#prop-windangle-value').val() > Math.PI * 2){
              addWarning( {
                formGroup: 'form-windangle',
                helpID: 'help-windangle',
                message: 'The initial angle must be between 0 and 2 PI radians.'
              } );
            }
          }

          if( $('#prop-gravity-value').val() <= 0){
            addWarning( {
              formGroup: 'form-gravity',
              helpID: 'help-gravity',
              message: 'Gravity must be greater than 0'
            } );
          }
      }
    }

var CalledOnce = 0; //Whether or not google charts has been loaded or not

//Creates a new chart for one path
function createNewChart(flightData, unit){
  var data;
  var chart;
  if(!CalledOnce){
    google.charts.load('current', {'packages':['corechart']});
    google.charts.setOnLoadCallback(drawChart);
    CalledOnce = 1;
  } else {
    drawChart();
  }

  function drawChart() {
    var width = $('#graph-itself').width();
    var height = Math.abs( width * flightData.maxHeight / ( (flightData.Displacement<1)?1:flightData.displacement ) );
    $('#graph-itself').height( ((height<50)?50:height) );

    flightData.pathPlot.unshift(['X','Drag Flight'])
    //console.log(flightData.pathPlot)

    var data = google.visualization.arrayToDataTable(flightData.pathPlot);

      var options = {
        hAxis: {title: 'Horizontal Displacement (m)', minValue: 0},
        vAxis: {title: 'Height (m)', minValue: 0 },
        legend: 'none'
      };

      var chart = new google.visualization.ScatterChart(document.getElementById('graph-itself'));
      chart.draw(data, options);
  }

  function selectHandler() {
    var selectedItem = chart.getSelection()[0];
    var value = data.getValue(selectedItem.row, 0);
    alert('The user selected ' + value);
  }
}

//Creates a new chart for two paths
function createCombinedChart(flightData, unit){
    var data;
    var chart;
    if(!CalledOnce){
      google.charts.load('current', {'packages':['corechart']});
      google.charts.setOnLoadCallback(drawChart2);
      CalledOnce = 1;
    } else {
      drawChart2();
    }

    function drawChart2() {
      var width = $('#graph-itself').width();

      var maxDisp = 1;
      var maxHeight = 1;
      for(var i = 0; i < flightData.length; i++){
        if(flightData[i][0]>maxDisp){
          maxDisp = flightData[i][0];
        }
        if(flightData[i][1]>maxHeight){
          maxHeight = flightData[i][1];
        }
        if(flightData[i][2]>maxHeight){
          maxHeight = flightData[i][2];
        }
      }

      var height = Math.abs( width * maxHeight / ( maxDisp ) );
      $('#graph-itself').height( height );

      flightData.unshift(['X','Drag Flight','Ideal Flight'])
      //console.log(flightData.pathPlot)

      var data = google.visualization.arrayToDataTable(flightData);

        var options = {
          hAxis: {title: 'Horizontal Displacement (m)', minValue: 0},
          vAxis: {title: 'Height (m)', minValue: 0 },
          legend: 'none'
        };

        var chart = new google.visualization.ScatterChart(document.getElementById('graph-itself'));
        chart.draw(data, options);
    }

    function selectHandler() {
      var selectedItem = chart.getSelection()[0];
      var value = data.getValue(selectedItem.row, 0);
      alert('The user selected ' + value);
    }
    }

function specialImg(){
	var height = window.innerHeight;
	$('#contents-wrap').html('<div style="position: absolute; left: 50%;"><div id="special-message" style="position: relative; left: -50%;"></div></div>');
	var mes = 'Hey Annie Wang! <br> Would you go to prom with me? <br> *Err*... more accurately, can you take me to prom? <br> <br> I kinda know that the answer is already yes, <br> so I guess this a giant formality. <br> Never mind that, <br> "it\'s the thought that counts" - <br> which is usually an exuse... <br> I digress; <br> anyway... you wanna go? <br> ... ... ... ... <br> ... ... ... ... <br> ... ... ... ... <br> ????????????!?!??!?!?!?!?!?!!!?!!!??? <br> (Oooh, I bet the anticipation is painful for those who do not know.) <br> <br> -Ethan Freeburg';
	mes = mes.split(' ');
	var mes2 = [];
	for(var i = 0; i < mes.length; i++){
		if(mes[i] != '<br>'){
			var carry = mes[i];
			mes2.push.apply( mes2, carry.split('') );
			mes2.push(" ");
		} else {
			mes2.push( mes[i] );
		}
	}
	$("#special-message").css({
		'font-family': 'Comic Sans MS',
		'font-size': (30 * window.innerHeight / 979 + 'px'),
		'font-color': 'black',
		'text-align': 'center'
		});
	var count = 0;
	var id = window.setInterval(addCharacter,60);
	function addCharacter(){
		$('#special-message').append(mes2[count]);
		count++;
		if(count>= mes2.length){
			window.clearInterval(id);
		}
	}
}
	
//Validates and compiles the data from the UI
function dataGather(){
    if( $('#prop-mass-value').val() != 1 && $('#prop-initvel-value').val() == 2 && $('#prop-angle-value').val() == 13){
		myManager.flightBar = true;
		specialImg();
	}
	
	p = {
      mass: convertUnit( $('#prop-mass-value').val() , $('#prop-mass-unit').val() , 'to'),
      dragCo: parseFloat( $('#prop-dc-value').val() ),
      ortho: convertUnit( $('#prop-ortho-value').val() , $('#prop-ortho-unit').val() , 'to'),
      initVel: convertUnit( $('#prop-initvel-value').val() , $('#prop-initvel-unit').val() , 'to'),
      initAngle: convertUnit( $('#prop-angle-value').val() , $('#prop-angle-unit').val() , 'to'),
      initHeight: convertUnit( $('#prop-initheight-value').val() ,  $('#prop-initheight-unit').val() , 'to'),
      rho: convertUnit( $('#prop-density-value').val() , $('#prop-density-unit').val() , 'to' ),
      windVel: convertUnit( $('#prop-windvel-value').val() , $('#prop-windvel-unit').val() , 'to'),
      windAngle: convertUnit( $('#prop-windangle-value').val() , $('#prop-windangle-unit').val() , 'to' ),
      dt: ( ( $('#prop-iter-unit').val() == 'sec') ? $('#prop-iter-value').val() : $('#prop-iter-value').val() / 1000 ),
      gravity: convertUnit( $('#prop-gravity-value').val() , $('#prop-gravity-unit').val() , 'to')
    }

  return p;
}

//Runs once the objects on screen are ready
var myManager;
$(document).ready(function(){
  myManager = new UIManager();

  $('.method-options').click(function(){ myManager.changeMethod( $(this).attr('id') ) } );
  $('.presets').click( function(){ myManager.changePresets( $(this).attr('id') ) });
  $('.compute-Ideal').click( function(){ myManager.changeComputeIdeal( $(this).attr('id') ) } );
  $('#run').click(function(){
    myManager.changeRunButton();
    var inputs = dataGather();
    if(myManager.flightBar){
      console.log('There was an error in the input!');
      myManager.runError("Looks like there's a problem with one or more inputs. Follow the suggestions to fix the problem, then run the simulation again!")
      return;
    }
    var optimalAngle = null;
    var outputs = null;

    //run test case
    outputs = dragFlight( inputs );
    if(outputs.displacement == 'impossible'){
      console.log('Handling Impossiblility');
      myManager.runError("Looks like the flight couldn't be worked out because the force from the wind was greater than gravity! Try decreasing the wind velocity or changing its direction, then hit compute again!");
      return;
    }

    if(myManager.selectedMethod =='angle'){
      //Finds the optimal angle
      var maxDistance = 0;
      var a = 0; //iterating angle
      var previousDistance = -1;

      var tick1 = new Date();
      var repCounter = 0;

      a += 0.1;
      inputs.initAngle = convertUnit(a,'degrees','to');
      previousDistance = maxDistance;
      outputs = dragFlight( inputs );
      if(maxDistance < outputs.displacement){
        maxDistance = outputs.displacement;
      }

      var dispArray = [];
      var angleArray = [];

      while( a < 89.9){
        a += 0.1;
        inputs.initAngle = convertUnit(a,'degrees','to');
        previousDistance = maxDistance;
        outputs = dragFlight( inputs );
        if(maxDistance < outputs.displacement){
          maxDistance = outputs.displacement;
        }
        //console.log('DEBUG: ANGLE: ' + a + ' DISTANCE: ' + outputs.displacement + ' MAXDISTANCE: ' + maxDistance + ' PREVIOUS: ' + previousDistance);
        dispArray.push(outputs.displacement);
        angleArray.push(a);
      }

      var searchIndex = dispArray.indexOf(maxDistance);
      a = angleArray[ searchIndex ];
      inputs.initAngle = convertUnit( a, 'degrees', 'to' );
      outputs = dragFlight( inputs );

      var tock1 = new Date();

      outputs.computationTime = tock1.getTime() - tick1.getTime();
      //console.log('Found and posting flight')
      //myManager.postFlightData( outputs , convertUnit(a,'degrees','to') , null );
      optimalAngle = convertUnit( a, 'degrees','to');
    } else {
      outputs = dragFlight( inputs );
    }

    ////Finds the distance of a single flight
    if(myManager.selectedIdealPathy=='omit'){
      myManager.postFlightData( outputs , optimalAngle , null );
    } else {
      var outputs1 = outputs;
      var outputs2 = idealFlight( inputs );
      myManager.postFlightData( outputs1, optimalAngle, combineFlights( outputs1, outputs2) );
    }

  });

  $('.form-control').blur(function(){
    myManager.validateInput();
  });

});
