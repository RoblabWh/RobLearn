import tensorflow as tf

#Importiere MNIST-Daten
from tensorflow.examples.tutorials.mnist import input_data
mnist = input_data.read_data_sets("temp/data/", one_hot=True)

#Definiere einige Parameter
element_size = 28 #Länge jedes Vektors in der Sequenz
time_steps = 28 #Anzahl der Elemente in einer Sequenz
num_classes = 10
batch_size = 128
hidden_layer_size = 128

#Hier werden Zusammenfassungen von TensorBoard gespeichert
LOG_DIR = "logs/RNN_with_summaries"

#Erstelle Platzhalter für Eingaben und Labels
_inputs = tf.placeholder(tf.float32, shape=[None, time_steps, element_size], name='inputs')
y = tf.placeholder(tf.float32, shape=[None,num_classes], name='labels')

batch_x, batch_y = mnist.train.next_batch(batch_size)
#Forme die Daten in 28 Sequenzen mit 28 Pixeln um (gleiche shape wie inputs)
batch_x = batch_x.reshape ((batch_size, time_steps, element_size))

#Diese Hilfsfunktion aus der TensorFlow-Dokumentation
#fügt einige Operationen zum Loggen der Zusammenfassung hinzu,
def variable_summaries(var):
    with tf.name_scope('summaries'):
        mean = tf.reduce_mean(var)
        tf.summary.scalar('mean', mean)
        with tf.name_scope('stddev'):
            stddev = tf.sqrt(tf.reduce_mean(tf.square(var - mean)))
        tf.summary.scalar('stddev', stddev)
        tf.summary.scalar('max', tf.reduce_max(var))
        tf.summary.scalar('min', tf.reduce_min(var))
        tf.summary.histogram('histogram', var)

#Gewichte und Bias für die Eingabe und die verborgene Schicht
with tf.name_scope('rnn_weights'):
    with tf.name_scope("W_x"):
        Wx = tf.Variable(tf.zeros([element_size, hidden_layer_size]))
        variable_summaries(Wx)
    with tf.name_scope("W_h"):
        Wh = tf.Variable(tf.zeros([hidden_layer_size, hidden_layer_size]))
        variable_summaries(Wh)
    with tf.name_scope("Bias"):
        b_rnn = tf.Variable(tf.zeros([hidden_layer_size]))
        variable_summaries(b_rnn)

def rnn_step(previous_hidden_state, x):
    current_hidden_state = tf.tanh(tf.matmul(previous_hidden_state, Wh) + tf.matmul(x, Wx) + b_rnn)
    return current_hidden_state

#Vorbereitungen der Eingaben für die Verwendung mit der Funktion scan
#Gestalt der Eingabedaten: (batch_size, time_steps, element_size)
processed_input = tf.transpose(_inputs, perm=[1, 0, 2])
#Neue Gestalt der Eingabedaten: (time_steps, batch_size, element_size)

initial_hidden = tf.zeros([batch_size, hidden_layer_size])
#Alle zustandsvektoren über die Zeit abfragen
all_hidden_states = tf.scan(rnn_step, processed_input, initializer=initial_hidden, name='states')


#Gewichte für die Ausgabeschichten
with tf.name_scope('linear_layer_weights') as scope:
    with tf.name_scope("W_linear"):
        Wl = tf.Variable(tf.truncated_normal([hidden_layer_size, num_classes], mean=0, stddev=.01))
        variable_summaries(Wl)
    with tf.name_scope("Bias_linear"):
        bl = tf.Variable(tf.truncated_normal([num_classes], mean=0, stddev=.01))
        variable_summaries(bl)

#Wende die lineare Schicht auf den Zustandsvektor an
def get_linear_layer(hidden_state):
    return tf.matmul(hidden_state, Wl) + bl

with tf.name_scope('linear_layer_weights') as scope:
    #Iteriere über die Zeit, wende die lineare Schicht auf alle Ausgaben des RNN an
    all_outputs = tf.map_fn(get_linear_layer, all_hidden_states)
    #Verwende die letzte Ausgabe
    output = all_outputs[-1]
    tf.summary.histogram('outputs', output)

with tf.name_scope('cross_entropy'):
    cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=output, labels=y))
    tf.summary.scalar('cross_entropy', cross_entropy)

with tf.name_scope('train'):
    #Verwenden des RMSPropOptimizer
    train_step = tf.train.RMSPropOptimizer(0.001, 0.9).minimize(cross_entropy)

with tf.name_scope('accuracy'):
    correct_prediction = tf.equal(tf.argmax(y, 1), tf.argmax(output, 1))
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32)) * 100
    tf.summary.scalar('accuracy', accuracy)

#Zusammenfassungen zusammenfügen
merged = tf.summary.merge_all()

#Erstelle einen kleinen Testdatensatz
test_data = mnist.test.images[:batch_size].reshape((-1, time_steps, element_size))

test_label = mnist.test.labels[:batch_size]

with tf.Session() as sess:
    #Schreibe die Zusammenfassungen für TensorBoard nach LOG_DIR
    train_writer = tf.summary.FileWriter(LOG_DIR + '/train', graph=tf.get_default_graph())
    test_writer = tf.summary.FileWriter(LOG_DIR + '/test', graph=tf.get_default_graph())

    sess.run(tf.global_variables_initializer())

    for i in range(10000):
        batch_x, batch_y = mnist.train.next_batch(batch_size)

        #Forme die Daten zu 28 Sequenzen mit je 28 Pixeln um
        batch_x = batch_x.reshape((batch_size, time_steps, element_size))

        summary, _ = sess.run([merged, train_step], feed_dict={_inputs:batch_x, y:batch_y})

        #Füge eine Zusammenfassung hinzu
        train_writer.add_summary(summary, i)

        if i % 1000 == 0:
            acc, loss, = sess.run([accuracy, cross_entropy], feed_dict={_inputs: batch_x, y:batch_y})
            print("Iter {0}, Verlust der Teilmenge= {1}, Genauigkeit Lerndaten{2}".format(str(i),
                                                                                          "{:.6f}".format(loss),
                                                                                          "{:.5f}".format(acc)))
        if i % 10:
            #Berechne die Genauigkeit für 128 MNIST-Testbilder und füge sie den Zusammenfassungen hinzu
            summary, acc = sess.run([merged, accuracy], feed_dict={_inputs:test_data, y:test_label})
            test_writer.add_summary(summary, i)

    test_acc = sess.run(accuracy, feed_dict={_inputs:test_data, y:test_label})

    print("Genauigkeit Test: ", test_acc)






