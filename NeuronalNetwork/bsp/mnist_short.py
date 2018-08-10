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

#Erstelle Platzhalter für Eingaben und Labels
_inputs = tf.placeholder(tf.float32, shape=[None, time_steps, element_size], name='inputs')
y = tf.placeholder(tf.float32, shape=[None,num_classes], name='labels')

#in Tensorflow eingebaute Funktionen
#rnn_cell = tf.contrib.rnn.BasicRnnCell(hidden_layer_size)
rnn_cell = tf.nn.rnn_cell.BasicRNNCell(hidden_layer_size)
outputs, _ = tf.nn.dynamic_rnn(rnn_cell, _inputs, dtype=tf.float32)

Wl = tf.Variable(tf.truncated_normal([hidden_layer_size, num_classes], mean=0, stddev=.01))

bl = tf.Variable(tf.truncated_normal([num_classes], mean=0, stddev=.01))

#Wende die lineare Schicht auf den Zustandsvektor an
def get_linear_layer(hidden_state):
    return tf.matmul(hidden_state, Wl) + bl

last_rnn_output = outputs[:,-1,:]
final_output = get_linear_layer(last_rnn_output)

softmax = tf.nn.softmax_cross_entropy_with_logits(logits=final_output, labels=y)

cross_entropy = tf.reduce_mean(softmax)
train_step = tf.train.RMSPropOptimizer(0.001, 0.9).minimize(cross_entropy)

correct_prediction = tf.equal(tf.argmax(y, 1), tf.argmax(final_output, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32)) * 100

sess = tf.InteractiveSession()
sess.run(tf.global_variables_initializer())

#Erstelle einen kleinen Testdatensatz
test_data = mnist.test.images[:batch_size].reshape((-1, time_steps, element_size))

test_label = mnist.test.labels[:batch_size]


for i in range(3001):
    batch_x, batch_y = mnist.train.next_batch(batch_size)

    #Forme die Daten zu 28 Sequenzen mit je 28 Pixeln um
    batch_x = batch_x.reshape((batch_size, time_steps, element_size))

    sess.run(train_step, feed_dict={_inputs:batch_x, y:batch_y})

    if i % 1000 == 0:
        acc, loss, = sess.run([accuracy, cross_entropy], feed_dict={_inputs: batch_x, y:batch_y})
        print("Iter {0}, Verlust der Teilmenge = {1}, Genauigkeit Lerndaten = {2}".format(str(i),
                                                                                      "{:.6f}".format(loss),
                                                                                      "{:.5f}".format(acc)))


test_acc = sess.run(accuracy, feed_dict={_inputs:test_data, y:test_label})

print("Genauigkeit Test: ", test_acc)






