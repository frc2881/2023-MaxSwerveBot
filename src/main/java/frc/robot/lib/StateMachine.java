// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

/**
 * A class for handling the infrastructure of a state machine.
 */
public class StateMachine {
  /**
   * An array of {@link Runnable} objects, one for each state in the state
   * machine, that handles the periodic running of the state.
   */
  private Runnable m_run[];

  /**
   * An array of {@link Runnable} objects, one for each state in the state
   * machine, that handles switching to the state.
   */
  private Runnable m_switchTo[];

  /**
   * An array of {@link Runnable} objects, one for each state in the state
   * machine, that handles switching from the state.
   */
  private Runnable m_switchFrom[];

  /**
   * The current state of the state machine.
   */
  private int m_state;

  /**
   * Creates a new state machine. The states must be numbered 0 through N - 1,
   * where N is the given number of states.
   *
   * @param numStates is the number of states in the state machine.
   *
   * @param defaultState is the default (initial) state.
   */
  public StateMachine(int numStates, int defaultState) {
    m_run = new Runnable[numStates];
    m_switchTo = new Runnable[numStates];
    m_switchFrom = new Runnable[numStates];

    m_state = defaultState;
  }

  /**
   * Adds a state handler to the state machine.
   *
   * @param state is the state.
   *
   * @param run is the {@link Runnable} used to handle execution of this state.
   */
  public void addState(int state, Runnable run) {
    m_run[state] = run;
    m_switchTo[state] = null;
    m_switchFrom[state] = null;
  }

  /**
   * Adds a state handler to the state machine.
   *
   * @param state is the state.
   *
   * @param run is the {@link Runnable} used to handle execution of this state.
   *
   * @param switchTo is the {@link Runnable} used to handle switching to this
   *                 state.
   */
  public void addState(int state, Runnable run, Runnable switchTo) {
    m_run[state] = run;
    m_switchTo[state] = switchTo;
    m_switchFrom[state] = null;
  }

  /**
   * Adds a state handler to the state machine.
   *
   * @param state is the state.
   *
   * @param run is the {@link Runnable} used to handle execution of this state.
   *
   * @param switchTo is the {@link Runnable} used to handle switching to this
   *                 state.
   *
   * @param switchFrom is the {@link Runnable} used to handle switching from
   *                   this state.
   */
  public void addState(int state, Runnable run, Runnable switchTo,
                       Runnable switchFrom) {
    m_run[state] = run;
    m_switchTo[state] = switchTo;
    m_switchFrom[state] = switchFrom;
  }

  /**
   * Runs the state handler for the current state of the state machine.
   */
  public void run() {
    m_run[m_state].run();
  }

  /**
   * Switches the state machine to a new state.
   *
   * @param state is the new state.
   */
  public void switchTo(int state) {
    if(m_switchFrom[m_state] != null) {
      m_switchFrom[state].run();
    }

    m_state = state;

    if(m_switchTo[state] != null) {
      m_switchTo[state].run();
    }
  }
}
