mir_knowledge base analyzer
=============================

Provides with information:

a) is there unfinished goals in knowledge base?
b) is there new predicates/knowledge in the knowledge base?

how to run?

You can open png file to see terminal arrange for testing this component

Basically you run the anaylizer node and you publish in 

        /pending_goals_analyzer/pending_goals/event_in
        
any information and then you get the result in the topic

        /pending_goals_analyzer/pending_goals/event_out

Done! now you should be able to get feedback wether the nowledge base has unfinished goals or

new knowledge
