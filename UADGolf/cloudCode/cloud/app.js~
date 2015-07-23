express = require('express');
app = express();

// Global app configuration section
app.set('views', 'cloud/views');  // Specify the folder to find templates
app.set('view engine', 'jade');    // Set the template engine
app.use(express.bodyParser());    // Middleware for reading request body

var views = require('cloud/views.js');
app.get('/', views.index);
app.get('/home', views.index);
app.get('/#', views.index);

app.listen();
