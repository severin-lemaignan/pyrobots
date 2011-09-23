#! /usr/bin/python
# -*- coding: utf-8 -*-

import logging
logging.basicConfig(level = logging.INFO)

import sys
import pygtk
pygtk.require('2.0')
import gtk
import gtk.glade

#from lowlevel import ActionPerformer
from dummylowlevel import ActionPerformer # For testing purposes

from helpers import places, postures

from actions import nav, look_at, configuration, manipulation

class ConnectDialog:
    
    def __init__(self):
        pass
        
    def run(self):
        self.widgets = gtk.glade.XML('res/laaswmctl.glade',"connect_dialog")
        dlg = self.widgets.get_widget('connect_dialog')
        
        dlg.run()
        port = self.widgets.get_widget('port').get_text()
        host = self.widgets.get_widget('host').get_text()
        
        dlg.destroy()
        
        return (host, port)
        
class NovelaCommander:
    def __init__(self):
        
        self.places = places.read()
        self.poses = postures.read()
        self.robot = ActionPerformer(['pr2c2', 'pr2c1'], 1235)
        
        self.istracking = False
        
        self.widgets = gtk.glade.XML('ui/novela.glade',"novela_main")
        events = { 'delete': self.delete,
                   'stopnav': self.stopnav,
                   'dock': self.dock,
                   'carry': self.carry,
                   'setpose': self.setpose,
                   'tuckedpose': self.tuckedpose,
                   'manippose': self.manippose,
                   'restpose': self.restpose,
                   'look': self.look,
                   'glance': self.glance,
                   'toggletracking': self.toggletracking,
                   'give': self.give,
                   'grab': self.grab}
                   
        self.widgets.signal_autoconnect(events)
        
        #self.connect_dlg = ConnectDialog()
        
        self.setplaces(self.widgets.get_widget("places_combobox1"))
        self.setplaces(self.widgets.get_widget("places_combobox2"))
        self.setplaces(self.widgets.get_widget("places_combobox3"))
        self.setplaces(self.widgets.get_widget("places_combobox4"))
        self.setplaces(self.widgets.get_widget("places_combobox5"))
        
        self.setpostures(self.widgets.get_widget("poses_combobox"))
        
        self.widgets.get_widget("novela_main").show_all()
    
    def setplaces(self, combobox):
        
        p = sorted(self.places.keys())
        self.set_model_from_list(combobox, p)
        combobox.set_active(0)
    
    def setpostures(self, combobox):
        
        p = sorted(self.poses.keys())
        self.set_model_from_list(combobox, p)
        combobox.set_active(0)
        
    def set_model_from_list (self, cb, items):
        """Setup a ComboBox or ComboBoxEntry based on a list of strings.
            (c) Thomas Hinkle
        """           
        model = gtk.ListStore(str)
        for i in items:
            model.append([i])
        cb.set_model(model)
        if type(cb) == gtk.ComboBoxEntry:
            cb.set_text_column(0)
        elif type(cb) == gtk.ComboBox:
            cell = gtk.CellRendererText()
            cb.pack_start(cell, True)
            cb.add_attribute(cell, 'text', 0)
        
    def delete(self, source=None, event=None):
        gtk.main_quit()
    def test(self):
        print("Toto")
    def execute(self, source=None, event=None):
        cmd = self.execute_dlg.run()
        print("Executing %s on remote"%(cmd))
        wm.execute(cmd)
        
    def stopnav(self, source=None, event=None):
        self.robot.execute(nav.cancel)

    def carry(self, source=None, event=None):
        print("Carry")
        dest = self.widgets.get_widget("places_combobox1").get_active_text()
        
        self.robot.execute(nav.carry, self.places[dest])
    
    def dock(self, source=None, event=None):
        dest = self.widgets.get_widget("places_combobox2").get_active_text()
        self.robot.execute(nav.gocloseto, self.places[dest])
    
    def setpose(self, source=None, event=None):
        pose = self.widgets.get_widget("poses_combobox").get_active_text()
        self.robot.execute(configuration.setpose, self.poses[pose])
        
    def tuckedpose(self, source=None, event=None):
        self.robot.execute(configuration.tuckedpose)
    def manippose(self, source=None, event=None):
        self.robot.execute(configuration.manipose)
    def restpose(self, source=None, event=None):
        self.robot.execute(configuration.restpose)
    
    def look(self, source=None, event=None):
        dest = self.widgets.get_widget("places_combobox3").get_active_text()
        self.robot.execute(look_at.look_at, self.places[dest])
        
    def glance(self, source=None, event=None):
        dest = self.widgets.get_widget("places_combobox4").get_active_text()
        self.robot.execute(look_at.glance_to, self.places[dest])
        
    def toggletracking(self, source=None, event=None):
        if self.istracking:
            self.istracking = False
            self.widgets.get_widget("places_combobox5").set_sensitive(True)
            self.widgets.get_widget("start_tracking_button").set_label("Start")
            self.robot.execute(look_at.stop_tracking)
        else:
            self.istracking = True
            dest = self.widgets.get_widget("places_combobox5").get_active_text()
            self.widgets.get_widget("places_combobox5").set_sensitive(False)
            self.widgets.get_widget("start_tracking_button").set_label("Stop") 
            self.robot.execute(look_at.track, self.places[dest])
    
    def grab(self, source=None, event=None):
        self.robot.execute(manipulation.basicgrab)
    
    def give(self, source=None, event=None):
        self.robot.execute(manipulation.basicgive)

       
if __name__ == '__main__':

            
    app = NovelaCommander()
    gtk.main()
