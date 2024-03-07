

import matplotlib.pyplot as plt

class InteractiveLegend(object):
    def __init__(self):
        self.legends = []
        self.figures = []
        self.lookup_artists = []
        self.lookup_handles = []

    def reset(self):
        self.legends = []
        self.figures = []
        self.lookup_artists = []
        self.lookup_handles = []

    def add(self, legend):
        self.legends.append(legend)

    def setup_and_wire(self):
        for legend in self.legends:
            self.figures.append(legend.axes.figure)
            lookup_artist, lookup_handle = self._build_lookups(legend)

            self.lookup_artists.append(lookup_artist)
            self.lookup_handles.append(lookup_handle)

        self._setup_connections()
        self.update()

    def _setup_connections(self):
        for legend in self.legends:
            for artist in legend.texts + legend.legendHandles:
                artist.set_picker(10) # 10 points tolerance

        for figs in self.figures:
            figs.canvas.mpl_connect('pick_event', self.on_pick)
            figs.canvas.mpl_connect('button_press_event', self.on_click)

    def _build_lookups(self, legend):
        labels = [t.get_text() for t in legend.texts]

        handles = legend.legendHandles
        label2handle = dict(zip(labels, handles))
        handle2text = dict(zip(handles, legend.texts))

        lookup_artist = {}
        lookup_handle = {}
        for artist in legend.axes.get_children():
            if artist.get_label() in labels:
                handle = label2handle[artist.get_label()]
                lookup_handle[artist] = handle
                lookup_artist[handle] = artist
                lookup_artist[handle2text[handle]] = artist

        lookup_handle.update(zip(handles, handles))
        lookup_handle.update(zip(legend.texts, handles))

        return lookup_artist, lookup_handle

    def on_pick(self, event):
        handle = event.artist
        for lookup_artist in self.lookup_artists:
            if handle in lookup_artist:
                artist = lookup_artist[handle]
                artist.set_visible(not artist.get_visible())
                self.update()

    def on_click(self, event):
        if event.button == 3:
            visible = False
        elif event.button == 2:
            visible = True
        else:
            return

        for lookup_artist in self.lookup_artists:
            for artist in lookup_artist.values():
                artist.set_visible(visible)
        self.update()

    def update(self):
        for idx, lookup_artist in enumerate(self.lookup_artists):
            for artist in lookup_artist.values():
                handle = self.lookup_handles[idx][artist]
                if artist.get_visible():
                    handle.set_visible(True)
                else:
                    handle.set_visible(False)
            self.figures[idx].canvas.draw()
