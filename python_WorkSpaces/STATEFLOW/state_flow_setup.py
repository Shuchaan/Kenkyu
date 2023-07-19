import graphviz
from transitions import Machine
from IPython.display import Image, display
from transitions.extensions import GraphMachine
from transitions.extensions.states import add_state_features, Error

class State_Flow_Setup:
    def __init__(self):
        self.states = ['Initialization', 'Search', 'Adjustment', 'Harvest', 'Pollination', 'Origin', "Next", 'Finish']
        self.transitions = [
            {'trigger': 'Init_to_Search', 'source': 'Initialization', 'dest': 'Search'}, 
            {'trigger': 'self', 'source':'Search', 'dest':'='},
            {'trigger': 'Search_to_Adjustment', 'source': 'Search', 'dest': 'Adjustment'}, 
            {'trigger': 'Adjustment_to_Harvest', 'source': 'Adjustment', 'dest': 'Harvest'}, 
            {'trigger': 'Harvest_to_Pollination', 'source': 'Harvest', 'dest': 'Pollination'}, 
            {'trigger': 'self_', 'source': 'Pollination', 'dest': 'Pollination'}, 
            {'trigger': 'Pollination_to_Origin', 'source': 'Pollination', 'dest': 'Origin'}, 
            {'trigger': 'Origin_to_Next', 'source': 'Origin', 'dest': 'Next'}, 
            {'trigger': 'Search_to_Finish', 'source': 'Search', 'dest': 'Finish'}, 
            {'trigger': 'Adjustment_to_Finish', 'source': 'Adjustment', 'dest': 'Finish'}, 
            {'trigger': 'Next_to_Search', 'source': 'Next', 'dest': 'Search'}, 
            {'trigger': 'Next_to_Adjustment', 'source': 'Next', 'dest': 'Adjustment'}, 
            {'trigger': 'Next_to_Finish', 'source': 'Next', 'dest': 'Finish'}, 
        ]  # 遷移の定義

        self.model = Matter()
        self.machine = GraphMachine(model=self.model, states=self.states, transitions=self.transitions, initial=self.states[0],
                        auto_transitions=False, ordered_transitions=True,
                        title='', show_auto_transitions=False, show_conditions=False,
                        finalize_event='action_output_graph', use_pygraphviz=False)
    
    def draw(self):
        self.model.get_graph().draw('test.png', prog='dot')  # 画像出力 ファイル名をここで指定
    
    def transition(self, flow):
        self.model.trigger(flow)
    
    def print_state(self):
        print(self.model.state)


class Matter(object):
    def __init__(self, filename=None):
        self.output = filename

    def action_output_graph(self, *args, **kwargs):
        dg = model.get_graph(*args, **kwargs).generate()
        if isinstance(self.output, str):
            graphviz.Source(dg, filename=self.output, format='png').render(cleanup=True)
        else:
            display(Image(dg.pipe(format='png')))