from flask import Flask, request, render_template
from flask_restful import Resource, Api

app = Flask(__name__)
api = Api(app)

todos = {}
global alimentar
alimentar = 'none'
@app.route("/", methods = ['GET','POST'])
def root(): 
	global alimentar	
	if request.method == 'POST':
		if request.form['submit']=='alimentar':

			alimentar = 'barriga cheia'

			return render_template("info.html", alimentar = alimentar)
	elif request.method == 'GET':	
 		return render_template('info.html', alimentar = alimentar)

#@app.route("/info", methods = ['GET','POST'])
#def root():
 #return render_template('info.html', name="nick")

#class HelloWorld(Resource):
#    def get(self):
#        return {'hello': 'world'}

#class TodoSimple(Resource):
#    def get(self, todo_id):
#        return {todo_id: todos[todo_id]}
#
#    def put(self, todo_id):
#        todos[todo_id] = request.form['data']
#        return {todo_id: todos[todo_id]}

#api.add_resource(TodoSimple, '/<string:todo_id>')
#api.add_resource(HelloWorld, '/')

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)
    #app.run(debug=True)

