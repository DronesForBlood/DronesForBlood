import npyscreen
import drawille
from dotmap import DotMap

# Globals
PREVIOUS_TERMINAL_WIDTH = None
PREVIOUS_TERMINAL_HEIGHT = None

class CustomBufferPager(npyscreen.BufferPager):
    '''
        Making custom MultiLineAction by adding the handlers
    '''
    def __init__(self,*args,**kwargs):
        super(CustomBufferPager,self).__init__(*args,**kwargs)
        # self.add_handlers({
        #     "q" : self._quit,
        # })


class BufferPagerWidget(npyscreen.BoxTitle):
    '''
        A framed widget containing multiline text
    '''
    _contained_widget = CustomBufferPager

class MultiLineWidget(npyscreen.BoxTitle):
    '''
        A framed widget containing multiline text
    '''
    _contained_widget = npyscreen.MultiLineEdit

class WindowForm(npyscreen.FormBaseNew):
    '''
        Frameless Form
    '''

    def create(self, *args, **kwargs):
        super(WindowForm, self).create(*args, **kwargs)


class MonitorGUI(npyscreen.NPSAppManaged):
    '''
        GUI class for Monitor GCS.
        This controls the rendering of the main window and acts as the registering point
        for all other widgets
    '''

    def __init__(self, stop_event):

        # Global stop event
        self.stop_event = stop_event

        # Main form
        self.window = None

        # Widgets
        self.overview = None
        self.armed_view = None
        self.gcs_status = None
        self.drone_status = None
        self.statustext_table = None
        self.actions = None

    def update(self, data):

        # check for resize
        terminal_width, terminal_height = drawille.getTerminalSize()
        if terminal_width != PREVIOUS_TERMINAL_WIDTH or terminal_height != PREVIOUS_TERMINAL_HEIGHT:
            # terminal size changed, updating gui
            self.window.erase()
            self.draw()

        #### Overview information ####
        row1 = "Last Heard    {2}{0}".format(data.last_heard,
                                            " " * int(4 * self.X_SCALING_FACTOR),
                                            " " * int(9 * self.X_SCALING_FACTOR))

        row2 = "Last System Status  {1}{0}".format(data.last_sys_status,
                                                   " " * int(4 * self.X_SCALING_FACTOR),
                                                   " " * int(9 * self.X_SCALING_FACTOR))

        self.overview.value = row1 + "\n" + row2
        self.overview.update(clear=True)

        #### ARMED ####
        row1 = "Base Mode: {2}{0}{3}{1}".format(data.base_mode,
                                                data.sub_mode,
                                                " " * int(4 * self.X_SCALING_FACTOR),
                                                " " * int(9 * self.X_SCALING_FACTOR))

        row2 = "Battery:  {2}{0}{3}{1}".format(data.battery,
                                                data.armed,
                                                " " * int(4 * self.X_SCALING_FACTOR),
                                                " " * int(9 * self.X_SCALING_FACTOR))

        self.armed_view.value = row1 + "\n" + row2
        self.armed_view.update(clear=True)

        ####  Drone Status ####
        row1 = "Current Position   {1}{0}".format(data.curr_pos,
                                                  " " * int(4 * self.X_SCALING_FACTOR))
        row2 = "Current Altitude   {1}{0}".format(data.curr_alt,
                                                  " " * int(4 * self.X_SCALING_FACTOR))
        row3 = "Target Position    {1}{0}".format(data.target_pos,
                                                  " " * int(4 * self.X_SCALING_FACTOR))
        row4 = "Distance To Target {1}{0}".format(data.dist_target,
                                                  " " * int(4 * self.X_SCALING_FACTOR))
        row5 = "\n"

        row6 = "Attitude {1}{0}".format(data.attitude,
                                        " " * int(4 * self.X_SCALING_FACTOR))

        self.drone_status.value = row1 + "\n" + row2 + "\n" + row3 + "\n" + row4 + "\n" + row5 + "\n" + row6
        self.drone_status.update(clear=True)

        #### GCS Status ####
        row1 = "Under development"
        self.gcs_status.value = row1
        self.gcs_status.update(clear=True)

        #### Status Text table ####
        self.statustext_table.entry_widget.buffer(data.statustext)
        self.statustext_table.entry_widget.update(clear=True)

        # update lazy updates at once
        self.window.display()

    def draw(self):
        # Setting the main window form
        self.window = WindowForm(parentApp=self, name="GCS Monitor")

        # Setting the terminal dimensions by querying the underlying curses library
        global PREVIOUS_TERMINAL_HEIGHT, PREVIOUS_TERMINAL_WIDTH
        max_y, max_x = self.window.curses_pad.getmaxyx()
        PREVIOUS_TERMINAL_HEIGHT = max_y
        PREVIOUS_TERMINAL_WIDTH = max_x

        # Minimum terminal size should be used for scaling
        self.Y_SCALING_FACTOR = float(max_y) / 27
        self.X_SCALING_FACTOR = float(max_x) / 104

        #####      Defaults            #######
        LEFT_OFFSET = 1
        TOP_OFFSET = 1

        #####      Overview widget     #######

        OVERVIEW_WIDGET_REL_X = LEFT_OFFSET
        OVERVIEW_WIDGET_REL_Y = TOP_OFFSET

        # equivalent to math.ceil =>  [ int(109.89) = 109 ]
        OVERVIEW_WIDGET_HEIGHT = int(5 * self.Y_SCALING_FACTOR)
        OVERVIEW_WIDGET_WIDTH = int(50 * self.X_SCALING_FACTOR)

        # draw overview widget
        self.overview = self.window.add(MultiLineWidget,
                                        name="Overview",
                                        relx=OVERVIEW_WIDGET_REL_X,
                                        rely=OVERVIEW_WIDGET_REL_Y,
                                        max_height=OVERVIEW_WIDGET_HEIGHT,
                                        max_width=OVERVIEW_WIDGET_WIDTH
                                        )
        self.overview.value = ""
        self.overview.entry_widget.editable = False

        ######    Armed widget  #########
        ARMED_WIDGET_REL_X = OVERVIEW_WIDGET_REL_X + OVERVIEW_WIDGET_WIDTH
        ARMED_WIDGET_REL_Y = OVERVIEW_WIDGET_REL_Y
        ARMED_WIDGET_HEIGHT = OVERVIEW_WIDGET_HEIGHT
        ARMED_WIDGET_WIDTH = OVERVIEW_WIDGET_WIDTH

        # draw armed widget
        self.armed_view = self.window.add(MultiLineWidget,
                                          name="Drone",
                                          relx=ARMED_WIDGET_REL_X,
                                          rely=ARMED_WIDGET_REL_Y,
                                          max_height=ARMED_WIDGET_HEIGHT,
                                          max_width=ARMED_WIDGET_WIDTH
                                          )
        self.armed_view.value = ""
        self.armed_view.entry_widget.editable = False

        ######    GCS Status widget  #########

        GCS_STATUS_WIDGET_REL_X = LEFT_OFFSET
        GCS_STATUS_WIDGET_REL_Y = OVERVIEW_WIDGET_REL_Y + OVERVIEW_WIDGET_HEIGHT
        GCS_STATUS_WIDGET_HEIGHT = int(10 * self.Y_SCALING_FACTOR)
        GCS_STATUS_WIDGET_WIDTH = int(50 * self.X_SCALING_FACTOR)

        # draw gcs_status widget
        self.gcs_status = self.window.add(MultiLineWidget,
                                          name="GCS Status",
                                          relx=GCS_STATUS_WIDGET_REL_X,
                                          rely=GCS_STATUS_WIDGET_REL_Y,
                                          max_height=GCS_STATUS_WIDGET_HEIGHT,
                                          max_width=GCS_STATUS_WIDGET_WIDTH
                                          )
        self.gcs_status.value = ""
        self.gcs_status.entry_widget.editable = False

        ######    Drone Status widget  #########

        DRONE_STATUS_WIDGET_REL_X = GCS_STATUS_WIDGET_REL_X + GCS_STATUS_WIDGET_WIDTH
        DRONE_STATUS_WIDGET_REL_Y = GCS_STATUS_WIDGET_REL_Y
        DRONE_STATUS_WIDGET_HEIGHT = GCS_STATUS_WIDGET_HEIGHT
        DRONE_STATUS_WIDGET_WIDTH = GCS_STATUS_WIDGET_WIDTH

        # draw drone status widget
        self.drone_status = self.window.add(MultiLineWidget,
                                            name="Drone Status",
                                            relx=DRONE_STATUS_WIDGET_REL_X,
                                            rely=DRONE_STATUS_WIDGET_REL_Y,
                                            max_height=DRONE_STATUS_WIDGET_HEIGHT,
                                            max_width=DRONE_STATUS_WIDGET_WIDTH
                                            )
        self.drone_status.value = ""
        self.drone_status.entry_widget.editable = False

        ######    Status Text widget  #########

        STATUSTEXT_WIDGET_REL_X = LEFT_OFFSET
        STATUSTEXT_WIDGET_REL_Y = DRONE_STATUS_WIDGET_REL_Y + DRONE_STATUS_WIDGET_HEIGHT
        STATUSTEXT_WIDGET_HEIGHT = int(8 * self.Y_SCALING_FACTOR)
        STATUSTEXT_WIDGET_WIDTH = int(100* self.X_SCALING_FACTOR)

        # draw status text widget
        self.statustext_table = self.window.add(BufferPagerWidget,
                                                name="Status Text",
                                                relx=STATUSTEXT_WIDGET_REL_X,
                                                rely=STATUSTEXT_WIDGET_REL_Y,
                                                max_height=STATUSTEXT_WIDGET_HEIGHT,
                                                max_width=STATUSTEXT_WIDGET_WIDTH
                                                )
        self.statustext_table.entry_widget.values = []
        self.statustext_table.entry_widget.scroll_exit = False
        self.drone_status.entry_widget.editable = False

        ######   Actions widget  #########

        # ACTIONS_WIDGET_REL_X = LEFT_OFFSET
        # ACTIONS_WIDGET_REL_Y = STATUSTEXT_WIDGET_REL_Y + STATUSTEXT_WIDGET_HEIGHT
        # self.actions = self.window.add(npyscreen.FixedText,
        #                                relx=ACTIONS_WIDGET_REL_X,
        #                                rely=ACTIONS_WIDGET_REL_Y
        #                                )
        #
        # self.actions.value = "q:Quit"
        # self.actions.display()
        # self.actions.editable = False

    def main(self):

        ######  THEMES  #########
        # 'elegant': npyscreen.Themes.ElegantTheme,
        # 'colorful': npyscreen.Themes.ColorfulTheme,
        # 'simple': npyscreen.Themes.DefaultTheme,
        # 'dark': npyscreen.Themes.TransparentThemeDarkText,
        # 'light': npyscreen.Themes.TransparentThemeLightText,
        # 'blackonwhite': npyscreen.Themes.BlackOnWhiteTheme

        npyscreen.setTheme(npyscreen.Themes.ElegantTheme)

        self.draw()